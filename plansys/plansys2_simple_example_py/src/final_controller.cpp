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

#include "std_msgs/msg/bool.hpp"
#include "zed/msg/structure_array.hpp"
#include "geometry_msgs/msg/point.hpp"

class Controller : public rclcpp::Node
{
public:
    Controller()
        : rclcpp::Node("controller"), state_(STARTING), exit_(false), counter(0), first(true), last("a"), people_in_loc(0), a(true), index(0), n(false), d(true), ppl_replanned(0), f_bat(true), w(true), moved_person(false), cons(false)
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
    }

    void init_knowledge()
    {
        problem_expert_->addInstance(plansys2::Instance{"spot", "robot"});
        problem_expert_->addInstance(plansys2::Instance{"a", "location"});
        problem_expert_->addInstance(plansys2::Instance{"b", "location"});
        problem_expert_->addInstance(plansys2::Instance{"c", "location"});
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
        problem_expert_->addPredicate(plansys2::Predicate("(connected c a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected a c)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected a battery_point)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected battery_point a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected exit a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected a exit)"));

        problem_expert_->addPredicate(plansys2::Predicate("(battery_unchecked spot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_free spot)"));

        goal = "and(searched spot a) (searched spot b)";
//        goal = "and(searched spot b)";
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

    void guide_person()
    {
        old_goal = goal;

        std::cout << "OLD GOAL: " << old_goal << std::endl;

        moved_person = true;
        executor_client_->cancel_plan_execution();
        std::string location;
        auto preds = problem_expert_->getPredicates();
        for (const auto& param : preds) {
            if (param.name == "next_move") {
                for (const auto& p : param.parameters) {
                    location = p.name;
                }
                std::cout << "asdf " << location << std::endl;
                problem_expert_->removePredicate(plansys2::Predicate("(next_move spot " + location + ")"));
            }
        }

        problem_expert_->addPredicate(plansys2::Predicate("(is_free spot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot " + current_position + ")"));
        problem_expert_->addPredicate(plansys2::Predicate("(battery_checked spot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(next_move spot exit)"));

        goal = "and (robot_at spot exit)";  //(finished spot)";
        problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        std::cout << domain << std::endl;
        std::cout << problem << std::endl;
        const auto& plan2 = plan.value();
        for (const auto& item : plan2.items) {
            std::cout << "Action: " << item.action << std::endl;
        }

        if (!plan.has_value())
        {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        }

        // Execute the plan
        executor_client_->start_plan_execution(plan.value());

  //       exit_ = true;

}

    void finish_plan()
    {
        executor_client_->cancel_plan_execution();
        std::string location;
        auto preds = problem_expert_->getPredicates();
        for (const auto &param : preds){
            if(param.name == "next_move"){
                for (const auto &p : param.parameters){
                    location = p.name;
                }
                std::cout << "asdf " << location << std::endl;
                problem_expert_->removePredicate(plansys2::Predicate("(next_move spot " + location + ")"));
            }
        }

        problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot " + current_position + ")"));
        problem_expert_->addPredicate(plansys2::Predicate("(next_move spot battery_point)"));

        // Replan
        //std::cout << "REMOVEEEDD" << std::endl;
        goal = "and (robot_at spot battery_point)";  //(finished spot)";
        problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));



        //            problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot a)"));
        //            problem_expert_->setGoal(plansys2::Goal("(and (robot_at spot c))"));

        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        std::cout << domain << std::endl;
        std::cout << problem << std::endl;

      const auto &plan2 = plan.value();
        for (const auto &item : plan2.items) {
            std::cout << "Action: " << item.action << std::endl;
        }

        if (!plan.has_value())
        {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        }

        // Execute the plan
        executor_client_->start_plan_execution(plan.value());

  //       exit_ = true;
    }


    void replan(std::string p)
    {
        executor_client_->cancel_plan_execution();

        auto a = check_predicate();
        //std::cout << "AA: " << a << std::endl;
        std::cout << "REPLANNING" << std::endl;

        if (a == false)
        {
            problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot " + current_position + ")"));
        }

     //        problem_expert_->removePredicate(plansys2::Predicate("(battery_unchecked spot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_free spot)"));
        // Replan

//        executor_client_->cancel_plan_execution();
        goal = goal + "(person_evaluated " + p + ") (person_reported " + p + ") (dialog_finished " + p + ")";

//        goal = goal + "(person_evaluated " + p + ")";
        problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

        //            problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot a)"));
        //            problem_expert_->setGoal(plansys2::Goal("(and (robot_at spot c))"));

        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        std::cout << domain << std::endl;
        std::cout << problem << std::endl;
      const auto &plan2 = plan.value();
        for (const auto &item : plan2.items) {
            std::cout << "Action: " << item.action << std::endl;
        }

        if (!plan.has_value())
        {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        }

        // Execute the plan
        executor_client_->start_plan_execution(plan.value());

        // exit_ = true;
    }


    void report_info()
    {
        auto array = zed::msg::StructureArray();
        for(const auto& ps : peopleStates)
        {
            auto msg = zed::msg::Structure();
            msg.person = ps.person;
            msg.state = ps.state;
            msg.consciousness = ps.consciousness;
            msg.location = ps.location;
            msg.coords = ps.coords;
            array.people_information.push_back(msg);
            //std::cout<<"B"<<std::endl;
        }

        //std::cout<<"PUBLISHINGGGGGGGGGGGGGG"<<std::endl;
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
        for (const auto &item : plan2.items) {
            std::cout << "Action: " << item.action << std::endl;
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
/*
            for(const auto& ps : peopleStates){
                std::cout << "Person: " << ps.person <<std::endl;
                std::cout << "State: " << ps.state <<std::endl;
                std::cout << "Response1: " << ps.response1 <<std::endl;
                std::cout << "Response2: " << ps.response2 <<std::endl;
                std::cout << "Response3: " << ps.response3 <<std::endl;
                std::cout << "Location: " << ps.location <<std::endl;
            }
*/
            for (const auto &action_feedback : feedback.action_execution_status)
            {
//                std::cout << "[" << action_feedback.action << " " <<
//                   action_feedback.completion * 100.0 << "%]";
                if(action_feedback.action == "report")
                {

                    std::string sent = action_feedback.message_status;
                    std::istringstream iss(sent);
                    std::string word;
                    std::vector<std::string> words;

                    while (iss >> word)
                    {
                        words.push_back(word);
                    }
                    //std::string a = words[0];
                    //std::cout << "WWW: " << a <<std::endl;

                    if (words.size() == 2)
                    {

                        //std::cout << reported << std::endl;
                        if (words[0] == "State" && !reported)
                        {
                            reported = true;
                            //std::cout << counter << " ppl " << people_in_loc << std::endl;
                            PersonState ps = peopleStates[counter-people_in_loc+ppl_replanned];
                            std::cout << "Person: " << ps.person <<std::endl;
                            std::cout << "State: " << ps.state <<std::endl;
                            std::cout << "Consciousness: " << ps.consciousness <<std::endl;
                            std::cout << "Location: " << ps.location <<std::endl;

                            if (index < people_in_loc) {
                                    //std::cout << "REPLAN WITH " << index << std::endl;
                                    std::string pers = people_in_loc_arr[index];
                                    index++;
                                    n = true;
                                    //w = true; ////// SI NO QUITAR
                                    ppl_replanned++;
                                    //people_in_loc_arr.erase(people_in_loc_arr.begin());
                                    replan(pers);
                            }
                            else{
                                   std::cout << "LOCATION FINISEHED " << std::endl;
                                   ppl_replanned = 0;
                                   if(cons == true){
                                       std::cout << "cons true! " << std::endl;
                                       guide_person();
                                   }
                                   cons = false;
                            }


                            //std::string person = "p" + std::to_string(counter-1);
                            //std::string predicate = "(person_reported " + person + ")";
                            //problem_expert_->addPredicate(plansys2::Predicate(predicate));
                        }
                       if(words[0] == "Report"){
                         //std::cout << "AAAAAAAAAAAAAAAAAAA"<< std::endl;
                         //auto msg = std_msgs::msg::Bool();
                         //msg.data = true;
                         //publisher_->publish(msg);
                       }
                    }
                }

                if(action_feedback.action == "dialog")
                {
                    std::string sent = action_feedback.message_status;
                    std::istringstream iss(sent);
                    std::string word;
                    std::vector<std::string> words;

                    while (iss >> word)
                    {
                        words.push_back(word);
                    }
                    //std::cout << "aaaaaaaaaaaaaaaaaaa " << words.size() <<std>

                    if (words.size() == 2)
                    {
                        //std::cout << "Aquiii " << words[0] << n << std::endl;
                        if (words[0] == "Diastate" && w)
                        {
                            w = false;
                            //std::string person = "p" + std::to_string(counter>
                            std::string person = people_in_loc_arr[index-1];
                            std::cout << "PERSON DIALOG " << person << " " << words[1] << std::endl;
                            std::string predicate = "(person_dialog " + person + " " + words[1] + ")";
                            problem_expert_->addPredicate(plansys2::Predicate(predicate));
                            peopleStates[counter-people_in_loc+ppl_replanned].consciousness = words[1];
                            reported = false;

                            if(words[1] == "conscious"){
                                std::cout << "cons true!  aaaaaaaaaaaaaa " << std::endl;
                                cons = true;
                            }

                            report_info();
                        }
                        //if (words[0] == "Person" && d)
                        //{   d = false;
                            //n = false;
                        //    std::cout << "pruebaaaaaaaaaaaaaaaaaaaaaaaaa " <<>
                        //}

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
                    //std::cout << words.size() <<std::endl;

                    if (words.size() == 3)
                    {
                        //std::cout << "Aquiii " << words[0] << n << std::endl;
                        if (words[0] == "Person" && n)
                        {
                            n = false;
                            //std::string person = "p" + std::to_string(counter-1);
                            std::string person = people_in_loc_arr[index-1];
                            std::cout << "PERSON STATE " << person << " " << words[2] << std::endl;
                            std::string predicate = "(person_state " + person + " " + words[2] + ")";
                            problem_expert_->addPredicate(plansys2::Predicate(predicate));
                            peopleStates[counter-people_in_loc+ppl_replanned].state = words[2];
                //            reported = false;
                            w = true;
                            report_info();
                        }
                        //if (words[0] == "Person" && d)
                        //{   d = false;
                            //n = false;
                        //    std::cout << "pruebaaaaaaaaaaaaaaaaaaaaaaaaa " << std::endl;
                        //}


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
//                          std::cout << words[0] << std::endl;
//                        if (places.find(words[2]) == places.end())
//                        {
                            if (words[0] == "Person")
                            {
                                //std::cout << "EY " << words[0] << " la pertson " << words[2] << " last " << last << std::endl;
                                 if(words[2] != last){
                                last = words[2];
                                
                                std::string in = words[3].substr(1, words[3].size() - 2);
                                std::stringstream ss(in);
                                std::string item;
                                float numbers[3];
                                int index = 0;
                                while(std::getline(ss, item, ',')){
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
                                //replan(person);
                            }
                            }
                            else if (words[0] == "No" && a)
                            {   a = false;
                                std::cout << "Finished, people found: " << people_in_loc << std::endl;
                                for (const auto& str : people_in_loc_arr) {
                                    std::cout << str << std::endl;
                                 }
                                report_info();
                                sleep(5);
                                index = 0;
                                if (!people_in_loc_arr.empty()) {
                                    std::string pers = people_in_loc_arr[index];
                                    index++;
                       d = true;
                                    n = true;
                                    //people_in_loc_arr.erase(people_in_loc_arr.begin());
                                    replan(pers);
                                }

                                //std::cout << "NO PERSON FOUND" << std::endl;
                                //places[words[2]] = false;
                            }

                            else if (words[0] == "Busqueda")
                            {
                                //a = true;
                                //a = false;
                                //std::cout << "Terminado, personas encontradas: " << people_in_loc << std::endl;
                            }
                           
//                        }
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
                            first = true; //SINO BORRAR
                            f_bat = true;
                            a = true;
                            people_in_loc = 0;
                            people_in_loc_arr.clear();

 
/*                           if(moved_person == true){
                                moved_person = false;
                                goal = old_goal;
                                problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

                                auto domain = domain_expert_->getDomain();
                                auto problem = problem_expert_->getProblem();
                                auto plan = planner_client_->getPlan(domain, problem);

                                std::cout << domain << std::endl;
                                std::cout << problem << std::endl;
                                const auto& plan2 = plan.value();
                                for (const auto& item : plan2.items) {
                                    std::cout << "Action: " << item.action << std::endl;
                                }

                                if (!plan.has_value())
                                {
                                    std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                                }

                                // Execute the plan
                                executor_client_->start_plan_execution(plan.value());


                            }*/
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
                            std::cout << "Low battery!" << action_feedback.completion << std::endl;
		            finish_plan();
                        }
                        else if(words[0] == "Enough" && f_bat == true)
                        {
                            f_bat = false;
                            std::cout << "Enough battery!" << action_feedback.completion << std::endl;
                        }
                    }
                }
            }

            if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
            {
                if (executor_client_->getResult().value().success)
                {
                    std::cout << "Successful finished " << std::endl;

if(moved_person == true){
                                moved_person = false;
                                goal = old_goal;
                                problem_expert_->setGoal(plansys2::Goal("(" + goal + ")"));

                                auto domain = domain_expert_->getDomain();
                                auto problem = problem_expert_->getProblem();
                                auto plan = planner_client_->getPlan(domain, problem);

                                std::cout << domain << std::endl;
                                std::cout << problem << std::endl;
                                const auto& plan2 = plan.value();
                                for (const auto& item : plan2.items) {
                                    std::cout << "Action: " << item.action << std::endl;
                                }

                                if (!plan.has_value())
                                {
                                    std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                                }

                                // Execute the plan
                                executor_client_->start_plan_execution(plan.value());


                            }

else{

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

    struct PersonState {
        std::string person;
        std::string state;
        std::string consciousness;
        std::string location;
        geometry_msgs::msg::Point coords;
    };

    bool exit_;
    bool first;
    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    int counter;
    std::string last;
    std::map<std::string, bool> places;
    std::string goal;
    std::string old_goal;
    std::string current_position;
    bool reported;
    int index;
    bool a;
    bool w;
    bool moved_person;
    bool cons;
bool d;
    bool f_bat;
    int ppl_replanned;
    bool n;
    int people_in_loc;
    std::vector<PersonState> peopleStates;
    rclcpp::Publisher<zed::msg::StructureArray>::SharedPtr publisher_;
    std::vector<std::string> people_in_loc_arr;

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
