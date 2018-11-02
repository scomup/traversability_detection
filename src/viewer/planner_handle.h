#ifndef HANDLER_FOR_PLANNER_H_
#define HANDLER_FOR_PLANNER_H_

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "../rrt/BiRRT.h"
#include "../rrt/path_optimizer.h"




class PlannerHandle
{
public:
  PlannerHandle(std::shared_ptr<RRT::StateSpace<Eigen::Vector3d>> state_space) {
    rrt_ = std::make_shared<RRT::BiRRT<Eigen::Vector3d>>(
        state_space,
        [](Eigen::Vector3d state) { size_t seed = 0;
            boost::hash_combine(seed, state.x());
            boost::hash_combine(seed, state.y());
            boost::hash_combine(seed, state.z());
            return seed; },
        3);
    rrt_->setGoalMaxDist(0.5);
    rrt_->setStepSize(0.5);
    rrt_->setMaxIterations(1e6);

    //path_opt_ = make()
  };


  std::shared_ptr<RRT::BiRRT<Eigen::Vector3d>> GetRRT(){return rrt_;};

  void setStart(const Eigen::Vector3d s){
    start_ = s;
  }


  void setGoal(const Eigen::Vector3d g){
    goal_ = g;
  }

  void run()
  {
    rrt_->reset();
    rrt_->setStartState(start_);
    rrt_->setGoalState(goal_);
    run_rrt_ =  new std::thread(&PlannerHandle::RunRRT, this);
  }

  void DrawStart() const
  {
    glColor3f(0.0f, 0.0f, 1.0f);
    drawSphere((GLfloat)start_.x(),(GLfloat)start_.y(),(GLfloat)start_.z(), 0.1);
  }

  void DrawGoal() const
  {
    glColor3f(0.5f, 0.0f, 0.5f);
    drawSphere((GLfloat)goal_.x(),(GLfloat)goal_.y(),(GLfloat)goal_.z(), 0.1);
  }

  void DrawPath() const
  {
    auto &start_tree = rrt_->startTree();
    auto &goal_tree = rrt_->goalTree();

    if (rrt_->startSolutionNode() != nullptr)
    {
      auto path = rrt_->getPath();
      glColor3f(0.0f, 0.0f, 0.0f);
      glLineWidth(5);
      glBegin(GL_LINE_STRIP);
      for (auto it = path.begin(); it != path.end(); it++)
      {
        glVertex3f(it->x(), it->y(), it->z());
      }
      glEnd();
    }

    glLineWidth(5);
    glColor3f(0.0f, 0.0f, 1.0f);
    for (const auto &node : start_tree.allNodes())
    {
      if (node.parent())
      {
        auto &p = node.parent()->state();
        auto &c = node.state();
        pangolin::glDrawLine(p.x(), p.y(), p.z(), c.x(), c.y(), c.z());
      }
    }

    glColor3f(0.5f, 0.0f, 0.5f);
    for (const auto &node : goal_tree.allNodes())
    {

      if (node.parent())
      {
        auto &p = node.parent()->state();
        auto &c = node.state();
        pangolin::glDrawLine(p.x(), p.y(), p.z(), c.x(), c.y(), c.z());
      }
    }
  }

private:

  void RunRRT()
  {
    rrt_->run();
  }

  inline void drawSphere(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat radius, GLfloat M = 10, GLfloat N = 10) const
{
  float step_z = M_PI / M;
  float step_xy = 2 * M_PI / N;
  float x[4], y[4], z[4];

  float angle_z = 0.0;
  float angle_xy = 0.0;
  int i = 0, j = 0;
  glBegin(GL_QUADS);
  for (i = 0; i < M; i++)
  {
    angle_z = i * step_z;

    for (j = 0; j < N; j++)
    {
      angle_xy = j * step_xy;

      x[0] = radius * sin(angle_z) * cos(angle_xy);
      y[0] = radius * sin(angle_z) * sin(angle_xy);
      z[0] = radius * cos(angle_z);

      x[1] = radius * sin(angle_z + step_z) * cos(angle_xy);
      y[1] = radius * sin(angle_z + step_z) * sin(angle_xy);
      z[1] = radius * cos(angle_z + step_z);

      x[2] = radius * sin(angle_z + step_z) * cos(angle_xy + step_xy);
      y[2] = radius * sin(angle_z + step_z) * sin(angle_xy + step_xy);
      z[2] = radius * cos(angle_z + step_z);

      x[3] = radius * sin(angle_z) * cos(angle_xy + step_xy);
      y[3] = radius * sin(angle_z) * sin(angle_xy + step_xy);
      z[3] = radius * cos(angle_z);

      for (int k = 0; k < 4; k++)
      {
        glVertex3f(xx + x[k], yy + y[k], zz + z[k]);
      }
    }
  }
  glEnd();
}

  std::shared_ptr<RRT::BiRRT<Eigen::Vector3d>> rrt_;
  //std::shared_ptr<RRT::PathOptimizer> path_opt_;
  Eigen::Vector3d start_;
  Eigen::Vector3d goal_;
  std::thread* run_rrt_;
};

#endif // HANDLER_FOR_BIRRT_H_