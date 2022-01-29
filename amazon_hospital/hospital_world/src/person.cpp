#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <chrono>
#include <vector>
#include <tuple>
#include <cmath>

const float PI = 3.14159265;
const int QUADRANTS = 4;

namespace gazebo
{
    class Person : public ModelPlugin
    {

    private:

        int state, current_wp;
        int dir_turn;
        static constexpr float lv_dt = 0.01;    // discrete lineal velocity
        static constexpr float av_dt = 0.003;   // discrete angular velocity 
        
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;

        // waypoints where (px, py, next_waypoint)
        std::vector <std::tuple<float, float, int>> wp;

        // quadrants vector to know the correct direction of turn
        std::vector <std::tuple<float, float>> quadrants;

    private:

        float GetDistanceEuclidean(float rx, float ry)
        {
            auto pose = this->model->WorldPose();
            
            return sqrt(pow(pose.Pos().X() - rx, 2) + pow(pose.Pos().Y() - ry, 2));
        }


        float GetAngle(float rx, float ry)
        {
            // Returns the Angle in Radians

            auto pose = this->model->WorldPose();
            float angle = atan2(abs(rx - pose.Pos().X()), abs(ry - pose.Pos().Y()));

            if (ry > pose.Pos().Y()) {
                angle = PI - angle;
            }
            if (rx < pose.Pos().X()) {
                angle *= -1;
            }
            return angle;
        }


        int GetBestTurnDirection(float desired_yaw, float actual_yaw)
        {
            auto get_quadrant = [](int yaw, std::vector<std::tuple<float, float>> & quadrants) {
                // iterate over all quadrants (4)
                for (std::size_t i = 0; i < QUADRANTS; i++) {
                    if (yaw >= std::get<0>(quadrants[i]) && yaw < std::get<1>(quadrants[i])) {
                        return i;
                    }
                }
            };

            // Search the corresponding quadrants
            int actual_quadrant = get_quadrant(actual_yaw, this->quadrants);
            int desired_quadrant = get_quadrant(desired_yaw, this->quadrants);

            // both angles share the same quadrant
            if (actual_quadrant == desired_quadrant) {
                return (desired_yaw > actual_yaw) ? 1 : -1; 
            }

            /** 
             * Calculate distance between angles in different quadrants
             * n1 -> quadrants of separation between the yaw angles for distance 1
             * n2 -> quadrants of separation between the yaw angles for distance 2
             **/
            int n1, n2, dist1, dist2;

            n1 = (desired_quadrant > actual_quadrant) ?
                  desired_quadrant - actual_quadrant :
                  QUADRANTS - abs((desired_quadrant - actual_quadrant) % QUADRANTS);
            n2 = QUADRANTS - n1;

            // Calculating Distance 1
            dist1 = std::get<1>(quadrants[actual_quadrant]) - actual_yaw;
            for (int i = 0; i < (n1 - 1); i++) {
                dist1 += PI/2;
            }
            dist1 += desired_yaw - std::get<0>(quadrants[desired_quadrant]);

            // Calculating Distance 2
            dist2 = actual_yaw - std::get<0>(quadrants[actual_quadrant]);
            for (int i = 0; i < (n2 - 1); i++) {
                dist2 += PI/2;
            }
            dist2 += std::get<1>(quadrants[desired_quadrant]) - desired_yaw;

            // Return best direction
            return (dist1 <= dist2) ? 1 : -1;
        }


        bool MoveToWaypoint(std::tuple<float, float, int> & waypoint)
        {
            static bool orientation_reached = false;
            static bool direction_chosen = false;

            float rx = std::get<0>(waypoint);
            float ry = std::get<1>(waypoint);
            float angle = GetAngle(rx, ry);
            auto pose = this->model->WorldPose();

            /**
             * Procedure for moving to the next waypoint
             **/
            // First, it will determine the orientation of turn
            if (!direction_chosen) {
                direction_chosen = true;
                dir_turn = GetBestTurnDirection(angle, pose.Rot().Yaw());
            }
            // Second, It will turn until Yaw desired is reached
            if (!orientation_reached) {
                pose.Rot() = ignition::math::Quaterniond(0, 0, pose.Rot().Yaw() + dir_turn*av_dt);
                if (abs(angle - pose.Rot().Yaw()) < 0.005) {
                    orientation_reached = true;
                }
            }
            // Third, it will move to position desired
            else {
                pose.Pos().X() += -lv_dt * (0*cos(pose.Rot().Yaw()) - 1*sin(pose.Rot().Yaw()));
                pose.Pos().Y() += -lv_dt * (0*sin(pose.Rot().Yaw()) + 1*cos(pose.Rot().Yaw()));
            }
            this->model->SetWorldPose(pose);

            if (orientation_reached && GetDistanceEuclidean(rx, ry) < 0.1) {
                // Reset static boolean values;
                orientation_reached = false;
                direction_chosen = false;
                return true;
            }
            return false;
        }


    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr)
        {
            this->model = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Person::OnUpdate, this, _1));

            std::cout << "Initial Position Person [" << this->model->WorldPose() << "]\n";

            // Setting WayPoints
            current_wp = 0;
            wp = {
                std::make_tuple(4, 6, 1),
                std::make_tuple(5, 3, 2),
                std::make_tuple(5, -14.5, 3),
                std::make_tuple(-5, -14.5, 4),
                std::make_tuple(-5, -25, 5),
                std::make_tuple(5, -25, 6),
                std::make_tuple(5, -14.5, 7),
                std::make_tuple(-5, -14.5, 8),
                std::make_tuple(-5, -1, 9),
                std::make_tuple(-4, 2, 10),
                std::make_tuple(-4, 5, 11),
                std::make_tuple(-2.5, 13, 12),
                std::make_tuple(3, 13, 13),
                std::make_tuple(4, 10, 0),
            };

            quadrants = {
                std::make_tuple(0.0, PI/2),
                std::make_tuple(PI/2, PI),
                std::make_tuple(-PI, -PI/2),
                std::make_tuple(-PI/2, 0.0)
            };
        }


        void OnUpdate(const common::UpdateInfo &)
        {
            if (MoveToWaypoint(wp[current_wp])) {
                current_wp = std::get<2>(wp[current_wp]);
            }
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(Person)
}