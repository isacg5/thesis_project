// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <map>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

class Controller : public rclcpp::Node
{
public:
    Controller()
        : rclcpp::Node("controller"), state_(STARTING), exit_(false), counter(0)
    {
    }

    void init()
    {
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        init_knowledge();
    }

    void init_knowledge()
    {
        problem_expert_->addInstance(plansys2::Instance{"spot", "robot"});
        problem_expert_->addInstance(plansys2::Instance{"a", "location"});
        problem_expert_->addInstance(plansys2::Instance{"b", "location"});
        problem_expert_->addInstance(plansys2::Instance{"c", "location"});

        problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected a b)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected b a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected c b)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected b c)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected c a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected a c)"));

        goal = "and(searched spot a)"; //(searched spot b)(searched spot c)";
//        goal = "and(robot_at spot b)";
        current_position = "a";
    }

    bool check_predicate()
    {
        auto predicated = problem_expert_->getPredicates();
        for (const auto &predicate : predicated) 
        {
            if (predicate.name == "robot_at"){
                return true;
            }
        }
        return false;
    }


    void replan(std::string p)
    {
        auto a = check_predicate();
        std::cout << "AA: " << a << std::endl;
        std::cout << "REPLANNING" << std::endl;

        if (a == false)
        {
            problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot " + current_position + ")"));
        }
        // Replan

        executor_client_->cancel_plan_execution();
        goal = goal + "(person_evaluated " + p + ")";
        problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

//            problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot a)"));
//            problem_expert_->setGoal(plansys2::Goal("(and (robot_at spot c))"));

        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        std::cout << domain << std::endl;
        std::cout << problem << std::endl;

        if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;

        }

        // Execute the plan
              executor_client_->start_plan_execution(plan.value());

   // exit_ = true;
    }

    void step()
    {
        switch (state_)
        {
        case STARTING:
        {
            // Set the goal for next state
            problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

            // Compute the plan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            std::cout << problem << std::endl;
            if (!plan.has_value())
            {
                std::cout << "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
            }

            // Execute the plan
            if (executor_client_->start_plan_execution(plan.value()))
            {
                state_ = SEARCH_A;
            }
        }
        break;

        case SEARCH_A:
        {
            auto feedback = executor_client_->getFeedBack();


            for (const auto &action_feedback : feedback.action_execution_status)
            {
                 std::cout << "[" << action_feedback.action << " " <<
                   action_feedback.completion * 100.0 << "%]";

                if (action_feedback.action == "search")
                {
                    std::string sent = action_feedback.message_status;
                    std::istringstream iss(sent);
                    std::string word;
                    std::vector<std::string> words;

                    while (iss >> word)
                    {
                        words.push_back(word);
                    }

                    if (words.size() == 3)
                    {
                        if (places.find(words[2]) == places.end())
                        {
                            if (words[0] == "Person")
                            {
                                std::string person = "p" + std::to_string(counter++);
                                std::cout << "PERSON FOUND "  << person << " in " << words[2] << std::endl;
                                std::string predicate = "(person_detected " + person + " " + words[2] + ")";
                                problem_expert_->addInstance(plansys2::Instance{person, "person"});
                                problem_expert_->addPredicate(plansys2::Predicate(predicate));
                                places[words[2]] = true;
                                replan(person);
                            }

                            else if (words[0] == "No")
                            {
                                std::cout << "NO PERSON FOUND" << std::endl;
                                places[words[2]] = false;
                            }
                        }
                    }
                }


                if (action_feedback.action == "move")
                {
                    std::string sent = action_feedback.message_status;
                    std::istringstream iss(sent);
                    std::string word;
                    std::vector<std::string> words;

                    while (iss >> word)
                    {
                        words.push_back(word);
                    }

                    if (words.size() == 2)
                    {
                        if (words[0] == "Moved" && current_position != words[1])
                        {
                            current_position = words[1];
                            std::cout << "Moved to " << words[1] << std::endl;
                        }
                    }
                }
            }

            if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
            {
                if (executor_client_->getResult().value().success)
                {
                    std::cout << "Successful finished " << std::endl;
                    state_ = FINISHED;
                }

                else
                {
                    for (const auto &action_feedback : feedback.action_execution_status)
                    {
                        if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED)
                        {
                            std::cout << "[" << action_feedback.action << "] finished with error: " << action_feedback.message_status << std::endl;
                        }
                    }
                }
            }
        }
        break;

        case FINISHED:
        {
            std::cout << "FINISHED" << std::endl;
            exit_ = true;
        }
        break;
        
        }
    }

    bool should_exit() const
    {
        return exit_;
    }

private:
    typedef enum
    {
        STARTING,
        SEARCH_A,
        FINISHED
    } StateType;
    StateType state_;

    bool exit_;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    int counter;
    std::map<std::string, bool> places;
    std::string goal;
    std::string current_position;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();

    node->init();

    rclcpp::Rate rate(5);
    while (rclcpp::ok() && !node->should_exit())
    {
        node->step();

        rate.sleep();
        rclcpp::spin_some(node->get_node_base_interface());
    }

    rclcpp::shutdown();

    return 0;
}
