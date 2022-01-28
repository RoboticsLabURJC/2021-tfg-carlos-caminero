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

        int state;
        static constexpr float dt = 0.001;
        
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;

        // waypoints where (px, py, next_waypoint)
        std::map <int, std::tuple<float, float, int> > wp = {
            {1, std::make_tuple(4, 6, 2)},
            {2, std::make_tuple(5, 3, 3)},
            {3, std::make_tuple(5, -14.5, 4)},
            {4, std::make_tuple(-5, -14.5, 5)},
            {5, std::make_tuple(-5, -1, 1)}
        };

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
            
            return atan2(ry - pose.Pos().Y(), rx - pose.Pos().X()) * 180 / PI;
        }


        bool MoveToWaypoint(std::tuple<float, float, int> & waypoint)
        {
            float rx = std::get<0>(waypoint);
            float ry = std::get<1>(waypoint);
            float angle = this->GetAngle(rx, ry);
            auto pose = this->model->WorldPose();

            pose.Rot().Euler(0, 0, angle);
            pose.Pos().X() += -dt * (0*cos(pose.Rot().Yaw()) - 1*sin(pose.Rot().Yaw()));
            pose.Pos().Y() += -dt * (0*sin(pose.Rot().Yaw()) + 1*cos(pose.Rot().Yaw()));

            this->model->SetWorldPose(pose);

            return this->GetDistanceEuclidean(rx, ry) < 0.025;
        }


    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr)
        {
            this->model = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Person::OnUpdate, this, _1));

            std::cout << "Loading person to follow at " << this->model->WorldPose() << std::endl;

            // Init values
            this->state = 1;
        }


        void OnUpdate(const common::UpdateInfo &)
        {
            if (this->MoveToWaypoint(wp[this->state])) {
                this->state = std::get<2>(wp[this->state]);
            }
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(Person)
}