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
#include <chrono>
#include <vector>


#include "std_msgs/msg/bool.hpp"
#include "zed/msg/structure_array.hpp"
#include "geometry_msgs/msg/point.hpp"

class Controller : public rclcpp::Node
{
public:
    Controller()
        : rclcpp::Node("controller"), state_(STARTING), exit_(false), counter(0), first(true), last("a"), people_in_loc(0), a(true), index(0), n(false), ppl_replanned(0), f_bat(true), w(true), moved_person(false), cons(false)
    {
        publisher_ = this->create_publisher<zed::msg::StructureArray>("/reported_info", 10);
    }

    void init()
    {
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        init_knowledge();
        init_time = std::chrono::high_resolution_clock::now();
    }

    void init_knowledge()
    {
        problem_expert_->addInstance(plansys2::Instance{"spot", "robot"});
        problem_expert_->addInstance(plansys2::Instance{"a", "location"});
        problem_expert_->addInstance(plansys2::Instance{"b", "location"});
        problem_expert_->addInstance(plansys2::Instance{"c", "location"});
        problem_expert_->addInstance(plansys2::Instance{"d", "location"});
        problem_expert_->addInstance(plansys2::Instance{"e", "location"});
        problem_expert_->addInstance(plansys2::Instance{"f", "location"});
        problem_expert_->addInstance(plansys2::Instance{"g", "location"});
        problem_expert_->addInstance(plansys2::Instance{"h", "location"});
        problem_expert_->addInstance(plansys2::Instance{"i", "location"});
        problem_expert_->addInstance(plansys2::Instance{"j", "location"});
        problem_expert_->addInstance(plansys2::Instance{"battery_point", "location"});
        problem_expert_->addInstance(plansys2::Instance{"exit", "location"});

        problem_expert_->addInstance(plansys2::Instance{"stand", "state"});
        problem_expert_->addInstance(plansys2::Instance{"lay", "state"});
        problem_expert_->addInstance(plansys2::Instance{"sit", "state"});
        problem_expert_->addInstance(plansys2::Instance{"unknown", "state"});

        problem_expert_->addInstance(plansys2::Instance{"conscious", "consciousness"});
        problem_expert_->addInstance(plansys2::Instance{"unconscious", "consciousness"});
        problem_expert_->addInstance(plansys2::Instance{"confused", "consciousness"});

        problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected a b)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected b a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected c b)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected b c)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected c d)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected d c)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected e c)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected c e)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected e h)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected h e)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected b e)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected e b)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected f d)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected d f)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected f i)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected i f)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected g a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected a g)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected h g)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected g h)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected i e)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected e i)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected j exit)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected exit j)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected a battery_point)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected battery_point a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected exit a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected a exit)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected exit battery_point)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected battery_point exit)"));

        problem_expert_->addPredicate(plansys2::Predicate("(battery_unchecked spot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_free spot)"));

        goal = "and(searched spot a) (searched spot b) (searched spot c) (searched spot g) (searched spot h)";
        //goal = "and(searched spot a) (searched spot b) (searched spot c) (searched spot d) (searched spot e) (searched spot f) (searched spot g) (searched spot h) (searched spot i) (searched spot j)";
        current_position = "a";
    }


    bool check_predicate()
    {
        auto predicated = problem_expert_->getPredicates();
        for (const auto &predicate : predicated)
        {
            if (predicate.name == "robot_at")
            {
                return true;
            }
        }
        return false;
    }


    bool haveSameValues(const std::vector<std::string>& vec1, const std::vector<std::string>& vec2)
    {
        if (vec1.size() != vec2.size())
        {
            return false;
        }

        std::vector<std::string> sortedVec1 = vec1;
        std::vector<std::string> sortedVec2 = vec2;

        std::sort(sortedVec1.begin(), sortedVec1.end());
        std::sort(sortedVec2.begin(), sortedVec2.end());
        return std::equal(sortedVec1.begin(), sortedVec1.end(), sortedVec2.begin());
    }

    bool is_finished()
    {
        std::vector<std::string> searched_locs;
        auto preds = problem_expert_->getPredicates();
        for (const auto &param : preds)
        {
            if (param.name == "searched")
            {
                for (const auto &p : param.parameters)
                {
                    if(p.name != "spot")
                    {
                        searched_locs.push_back(p.name);
                    }
                }
            }
        }
        std::vector<std::string> values;
        std::istringstream ss(old_goal);
        std::string word;

        while (ss >> word)
        {
            if (word == "(searched" || word == "and(searched")
            {
                std::string spot;
                std::string value;
                ss >> spot >> value;
                auto pos = value.find(')');
                std::string final_value = value.substr(0, pos);
                values.push_back(final_value);
            }
        }

        //for (const std::string& val : values)
        //{
        //    std::cout << "Valor: " << val << std::endl;
        //}

        std::cout << haveSameValues(values, searched_locs) << std::endl;

        return haveSameValues(values, searched_locs);
    }

    void guide_person()
    {
        old_goal = goal;

        std::cout << "OLD GOAL: " << old_goal << std::endl;

        auto replan_init_time = std::chrono::high_resolution_clock::now();

        moved_person = true;
        executor_client_->cancel_plan_execution();
        std::string location;
        auto preds = problem_expert_->getPredicates();
        for (const auto &param : preds)
        {
            if (param.name == "next_move")
            {
                for (const auto &p : param.parameters)
                {
                    location = p.name;
                }
                problem_expert_->removePredicate(plansys2::Predicate("(next_move spot " + location + ")"));
            }
        }

        problem_expert_->addPredicate(plansys2::Predicate("(is_free spot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot " + current_position + ")"));
        problem_expert_->addPredicate(plansys2::Predicate("(battery_checked spot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(next_move spot exit)"));

        goal = "and (robot_at spot exit)";
        problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        std::cout << problem << std::endl;

        const auto &plan2 = plan.value();
        for (const auto &item : plan2.items)
        {
            std::cout << "Action: " << item.action << std::endl;
        }

        if (!plan.has_value())
        {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        }

        auto replan_end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> replan_duration = replan_end_time - replan_init_time;
        std::cout << "Time taken for replanning: " << replan_duration.count() << " milliseconds" << std::endl;

        executor_client_->start_plan_execution(plan.value());
    }


    void finish_plan()
    {
        auto replan_init_time = std::chrono::high_resolution_clock::now();

        executor_client_->cancel_plan_execution();
        std::string location;
        auto preds = problem_expert_->getPredicates();
        for (const auto &param : preds)
        {
            if (param.name == "next_move")
            {
                for (const auto &p : param.parameters)
                {
                    location = p.name;
                }
                problem_expert_->removePredicate(plansys2::Predicate("(next_move spot " + location + ")"));
            }
        }

        problem_expert_->addPredicate(plansys2::Predicate("(is_free spot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot " + current_position + ")"));
        problem_expert_->addPredicate(plansys2::Predicate("(next_move spot battery_point)"));

        // Replan
        goal = "and (robot_at spot battery_point)"; //(finished spot)";
        problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        std::cout << problem << std::endl;

        const auto &plan2 = plan.value();
        for (const auto &item : plan2.items)
        {
            std::cout << "Action: " << item.action << std::endl;
        }

        if (!plan.has_value())
        {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        }

        auto replan_end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> replan_duration = replan_end_time - replan_init_time;
        std::cout << "Time taken for replanning: " << replan_duration.count() << " milliseconds" << std::endl;

        executor_client_->start_plan_execution(plan.value());
    }


    void replan(std::string p)
    {
        auto replan_init_time = std::chrono::high_resolution_clock::now();

        executor_client_->cancel_plan_execution();

        auto a = check_predicate();
        std::cout << "Replanning" << std::endl;

        if (a == false)
        {
            problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot " + current_position + ")"));
        }

        problem_expert_->addPredicate(plansys2::Predicate("(is_free spot)"));

        // Replan
        goal = goal + "(person_evaluated " + p + ") (person_reported " + p + ") (dialog_finished " + p + ")";

        problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        std::cout << problem << std::endl;

        const auto &plan2 = plan.value();
        for (const auto &item : plan2.items)
        {
            std::cout << "Action: " << item.action << std::endl;
        }

        if (!plan.has_value())
        {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        }

        auto replan_end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> replan_duration = replan_end_time - replan_init_time;
        std::cout << "Time taken for replanning: " << replan_duration.count() << " milliseconds" << std::endl;

        executor_client_->start_plan_execution(plan.value());
    }


    void report_info()
    {
        reported = false;
        auto array = zed::msg::StructureArray();
        for (const auto &ps : peopleStates)
        {
            auto msg = zed::msg::Structure();
            msg.person = ps.person;
            msg.state = ps.state;
            msg.consciousness = ps.consciousness;
            msg.location = ps.location;
            msg.coords = ps.coords;
            array.people_information.push_back(msg);
        }

        publisher_->publish(array);
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

            if (!plan.has_value())
            {
                std::cout << "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
            }

            const auto &plan2 = plan.value();
            for (const auto &item : plan2.items)
            {
                std::cout << "Action: " << item.action << std::endl;
            }

            // Execute the plan
            if (executor_client_->start_plan_execution(plan.value()))
            {
                state_ = INIT;
            }
        }
        break;

        case INIT:
        {
            auto feedback = executor_client_->getFeedBack();
            for (const auto &action_feedback : feedback.action_execution_status)
            {
                if (action_feedback.action == "report")
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
                        if (words[0] == "State" && !reported)
                        {
                            reported = true;
                            PersonState ps = peopleStates[counter - people_in_loc + ppl_replanned];
                            std::cout << "Person: " << ps.person << std::endl;
                            std::cout << "State: " << ps.state << std::endl;
                            std::cout << "Consciousness: " << ps.consciousness << std::endl;
                            std::cout << "Location: " << ps.location << std::endl;

                            if (index < people_in_loc)
                            {
                                std::string pers = people_in_loc_arr[index];
                                index++;
                                n = true;
                                ppl_replanned++;
                                replan(pers);
                            }
                            else
                            {
                                ppl_replanned = 0;
                                if (cons == true)
                                {
                                    guide_person();
                                }
                                cons = false;
                            }
                        }
                    }
                }

                if (action_feedback.action == "dialog")
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
                        if (words[0] == "Diastate" && w)
                        {
                            w = false;
                            std::string person = people_in_loc_arr[index - 1];
                            std::cout << "PERSON DIALOG " << person << " " << words[1] << std::endl;
                            std::string predicate = "(person_dialog " + person + " " + words[1] + ")";
                            problem_expert_->addPredicate(plansys2::Predicate(predicate));
                            peopleStates[counter - people_in_loc + ppl_replanned].consciousness = words[1];

                            if (words[1] == "conscious")
                            {
                                cons = true;
                            }

                            report_info();
                        }
                    }
                }

                if (action_feedback.action == "evaluate")
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
                        if (words[0] == "Person" && n)
                        {
                            n = false;
                            std::string person = people_in_loc_arr[index - 1];
                            std::cout << "PERSON STATE " << person << " " << words[2] << std::endl;
                            std::string predicate = "(person_state " + person + " " + words[2] + ")";
                            problem_expert_->addPredicate(plansys2::Predicate(predicate));
                            peopleStates[counter - people_in_loc + ppl_replanned].state = words[2];
                            w = true;
                            report_info();
                        }
                    }
                }

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

                    if (words.size() == 4)
                    {
                        if (words[0] == "Person")
                        {
                            if (words[2] != last)
                            {
                                last = words[2];

                                std::string in = words[3].substr(1, words[3].size() - 2);
                                std::stringstream ss(in);
                                std::string item;
                                float numbers[3];
                                int index = 0;
                                while (std::getline(ss, item, ','))
                                {
                                    numbers[index++] = std::stof(item);
                                }

                                geometry_msgs::msg::Point pos;
                                pos.x = numbers[0];
                                pos.y = numbers[1];
                                pos.z = numbers[2];
                                std::string person = "p" + std::to_string(counter++);
                                peopleStates.push_back({person, "None", "None", words[1], pos});
                                std::cout << "PERSON FOUND " << person << " in " << words[1] << std::endl;
                                std::string predicate = "(person_detected " + person + " " + words[1] + ")";
                                problem_expert_->addInstance(plansys2::Instance{person, "person"});
                                problem_expert_->addPredicate(plansys2::Predicate(predicate));
                                places[words[1]] = true;
                                people_in_loc++;
                                people_in_loc_arr.push_back(person);
                            }
                        }
                        else if (words[0] == "No" && a)
                        {
                            a = false;
                            std::cout << "Finished, people found: " << people_in_loc << std::endl;
                            for (const auto &str : people_in_loc_arr)
                            {
                                std::cout << str << std::endl;
                            }
                            report_info();
                            sleep(5);
                            index = 0;
                            if (!people_in_loc_arr.empty())
                            {
                                std::string pers = people_in_loc_arr[index];
                                index++;
                                n = true;
                                replan(pers);
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
                            first = true;
                            f_bat = true;
                            a = true;
                            people_in_loc = 0;
                            people_in_loc_arr.clear();
                        }
                    }
                }

                if (action_feedback.action == "check")
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
                        if (words[0] == "Low" && first == true)
                        {
                            first = false;
                            std::cout << "Low battery: " << action_feedback.completion << std::endl;
                            finish_plan();
                        }
                        else if (words[0] == "Enough" && f_bat == true)
                        {
                            f_bat = false;
                            std::cout << "Enough battery: " << action_feedback.completion << std::endl;
                        }
                    }
                }
            }

            if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
            {
                if (executor_client_->getResult().value().success)
                {
                    std::cout << "Successful finished " << std::endl;

                    if (moved_person == true && !is_finished())
                    {
                        moved_person = false;
                        goal = old_goal;
                        problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

                        auto domain = domain_expert_->getDomain();
                        auto problem = problem_expert_->getProblem();
                        auto plan = planner_client_->getPlan(domain, problem);

                        std::cout << domain << std::endl;
                        std::cout << problem << std::endl;
                        const auto &plan2 = plan.value();
                        for (const auto &item : plan2.items)
                        {
                            std::cout << "Action: " << item.action << std::endl;
                        }

                        if (!plan.has_value())
                        {
                            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                        }

                        // Execute the plan
                        executor_client_->start_plan_execution(plan.value());
                    }

                    else
                    {
                        state_ = FINISHED;
                    }
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
            end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration_time = end_time - init_time;
            std::cout << "FINISHED. Time taken: " << duration_time.count() << " seconds" << std::endl;
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
        INIT,
        FINISHED
    } StateType;
    StateType state_;

    struct PersonState
    {
        std::string person;
        std::string state;
        std::string consciousness;
        std::string location;
        geometry_msgs::msg::Point coords;
    };

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    rclcpp::Publisher<zed::msg::StructureArray>::SharedPtr publisher_;
    std::map<std::string, bool> places;
    std::chrono::time_point<std::chrono::high_resolution_clock> init_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    std::vector<std::string> people_in_loc_arr;
    std::vector<PersonState> peopleStates;
    std::string goal;
    std::string old_goal;
    std::string current_position;
    std::string last;
    bool exit_;
    bool first;
    bool reported;
    bool n;
    bool a;
    bool w;
    bool moved_person;
    bool cons;
    bool f_bat;
    int index;
    int ppl_replanned;
    int people_in_loc;
    int counter;
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
