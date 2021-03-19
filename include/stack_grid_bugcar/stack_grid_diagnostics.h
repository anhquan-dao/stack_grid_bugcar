#ifndef STACK_GRID_DIAGNOSTICS
#define STACK_GRID_DIAGNOSTICS

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

namespace stack_grid_bugcar{
    
    class StackGridDiagnostics{
        public:
            StackGridDiagnostics(){
                diagnostics.add("Custom Stack Grid status", this, &StackGridDiagnostics::updateDiag);
                diagnostics.setHardwareID("none");
                mb_polls.store(0);
                layer_exc_time.store(0);
                last_call = ros::Time::now().toSec();
            }
            ~StackGridDiagnostics(){

            }
            void updateDiag(diagnostic_updater::DiagnosticStatusWrapper& stat){
                std::lock_guard<std::mutex> lg(data_mutex);
                mb_polling_rate = mb_polls.load()/(ros::Time::now().toSec() - last_call);
                layer_exc_frequency_ = mb_polls.load()/layer_exc_time;
                last_call = ros::Time::now().toSec();
                mb_polls.store(0);
                layer_exc_time.store(0);
                
                if(std::any_of(error_input_count.begin(), error_input_count.end(),
                            [](int i){return i == EMPTY_MSG_ERR;})){
                    
                    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Some input topics are empty or have not been published");
                }
                else if(std::any_of(error_input_count.begin(), error_input_count.end(),
                            [](int i){return i == LATE_UPDATE_ERR;})){
                    
                    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Some input topics have not been published for some time");
                }
                else if(abs(mb_polling_rate - map_update_frequency_)/map_update_frequency_ > 0.1){
                    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Polling rate is too fast/slow");
                }
                else{
                    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
                }
                
                error_input_count.clear();
                stat.add("Frequency of layer", layer_exc_frequency_);
                stat.add("Move_base polling rate", mb_polling_rate);
                stat.add("Costmap2D rate", map_update_frequency_);
            }
            void updateStatus(){
                diagnostics.update();
            }
            void updatePoll(){
                mb_polls.fetch_add(1);
                float temp = layer_exc_time.load();
                layer_exc_time.store(temp + (ros::Time::now().toSec()-layer_exc_start));
            }        
            void getCostmap2DFrequency(float update_frequency){
                map_update_frequency_ = update_frequency;
            }
            void getLayerStart(){
                layer_exc_start = ros::Time::now().toSec();
            }

        private:

            diagnostic_updater::Updater diagnostics;

            std::mutex data_mutex;

            std::atomic<int> mb_polls;
            float mb_polling_rate;
            float map_update_frequency_ = 0;
            std::atomic<float> layer_exc_time;
            float layer_exc_frequency_;
            float layer_exc_start = 0;
            float layer_exc_end = 0;
            float last_call = 0;

            std::vector<int> error_input_count;



    };

}



#endif