#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <chrono>
#include <vector>
#include <tuple>
#include <cmath>
#include <map>

#define PI 3.14159265

namespace gazebo
{
    class Person : public ModelPlugin
    {

    private:

        int state, current_wp;
        static constexpr float dt = 0.005;
        
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;

        // waypoints where (px, py, next_waypoint)
        std::map <int, std::tuple<float, float, int> > wp;

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


        bool MoveToWaypoint(std::tuple<float, float, int> & waypoint)
        {
            static bool orientation_reached = false;
            float rx = std::get<0>(waypoint);
            float ry = std::get<1>(waypoint);
            float angle = this->GetAngle(rx, ry);
            auto pose = this->model->WorldPose();

            // First, It will turn until Yaw desired is reached
            if (!orientation_reached) {
                pose.Rot() = ignition::math::Quaterniond(0, 0, pose.Rot().Yaw() + 0.003);
                if (abs(angle - pose.Rot().Yaw()) < 0.005) {
                    orientation_reached = true;
                }
            }
            // Move to position desired
            else {
                pose.Pos().X() += -dt * (0*cos(pose.Rot().Yaw()) - 1*sin(pose.Rot().Yaw()));
                pose.Pos().Y() += -dt * (0*sin(pose.Rot().Yaw()) + 1*cos(pose.Rot().Yaw()));
            }
            this->model->SetWorldPose(pose);

            if (orientation_reached && this->GetDistanceEuclidean(rx, ry) < 0.1) {
                orientation_reached = false;    // Initialize the next waypoint
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
            this->current_wp = 1;
            wp = {
                {1, std::make_tuple(4, 6, 2)},
                {2, std::make_tuple(5, 3, 3)},
                {3, std::make_tuple(5, -14.5, 4)},
                {4, std::make_tuple(-5, -14.5, 5)},
                {5, std::make_tuple(-5, -25, 6)},
                {6, std::make_tuple(5, -25, 7)},
                {7, std::make_tuple(5, -14.5, 8)},
                {8, std::make_tuple(-5, -14.5, 9)},
                {9, std::make_tuple(-5, -1, 10)},
                {10, std::make_tuple(-4, 2, 11)},
                {11, std::make_tuple(-4, 5, 12)},
                {12, std::make_tuple(-2.5, 13, 13)},
                {13, std::make_tuple(3, 13, 14)},
                {14, std::make_tuple(4, 10, 1)},
            };
        }


        void OnUpdate(const common::UpdateInfo &)
        {
            if (this->MoveToWaypoint(wp[this->current_wp])) {
                this->current_wp = std::get<2>(wp[this->current_wp]);
            }
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(Person)
}