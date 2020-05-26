/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame_joints_widget.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/move_group/capability_names.h>

#include <geometric_shapes/shape_operations.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <tf2_ros/buffer.h>

#include <std_srvs/srv/empty.hpp>

#include <QMessageBox>
#include <QInputDialog>
#include <QShortcut>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros_visualization.motion_planning_frame");

MotionPlanningFrame::MotionPlanningFrame(MotionPlanningDisplay* pdisplay, rviz_common::DisplayContext* context,
                                         QWidget* parent)
  : QWidget(parent)
  , planning_display_(pdisplay)
  , context_(context)
  , ui_(new Ui::MotionPlanningUI())
  , first_time_(true)
{
  node_ = planning_display_->getNode();
  clear_octomap_service_client_ = node_->create_client<std_srvs::srv::Empty>(move_group::CLEAR_OCTOMAP_SERVICE_NAME);

  // set up the GUI
  ui_->setupUi(this);
  // add more tabs
  joints_tab_ = new MotionPlanningFrameJointsWidget(planning_display_, ui_->tabWidget);
  ui_->tabWidget->addTab(joints_tab_, "Joints");
  connect(planning_display_, SIGNAL(queryStartStateChanged()), joints_tab_, SLOT(queryStartStateChanged()));
  connect(planning_display_, SIGNAL(queryGoalStateChanged()), joints_tab_, SLOT(queryGoalStateChanged()));

  // connect bottons to actions; each action usually registers the function pointer for the actual computation,
  // to keep the GUI more responsive (using the background job processing)
  connect(ui_->plan_button, SIGNAL(clicked()), this, SLOT(planButtonClicked()));
  connect(ui_->execute_button, SIGNAL(clicked()), this, SLOT(executeButtonClicked()));
  connect(ui_->plan_and_execute_button, SIGNAL(clicked()), this, SLOT(planAndExecuteButtonClicked()));
  connect(ui_->stop_button, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
  connect(ui_->start_state_combo_box, SIGNAL(activated(QString)), this, SLOT(startStateTextChanged(QString)));
  connect(ui_->goal_state_combo_box, SIGNAL(activated(QString)), this, SLOT(goalStateTextChanged(QString)));
  connect(ui_->planning_group_combo_box, SIGNAL(currentIndexChanged(QString)), this,
          SLOT(planningGroupTextChanged(QString)));
  connect(ui_->database_connect_button, SIGNAL(clicked()), this, SLOT(databaseConnectButtonClicked()));
  connect(ui_->save_scene_button, SIGNAL(clicked()), this, SLOT(saveSceneButtonClicked()));
  connect(ui_->save_query_button, SIGNAL(clicked()), this, SLOT(saveQueryButtonClicked()));
  connect(ui_->delete_scene_button, SIGNAL(clicked()), this, SLOT(deleteSceneButtonClicked()));
  connect(ui_->delete_query_button, SIGNAL(clicked()), this, SLOT(deleteQueryButtonClicked()));
  connect(ui_->planning_scene_tree, SIGNAL(itemSelectionChanged()), this, SLOT(planningSceneItemClicked()));
  connect(ui_->load_scene_button, SIGNAL(clicked()), this, SLOT(loadSceneButtonClicked()));
  connect(ui_->load_query_button, SIGNAL(clicked()), this, SLOT(loadQueryButtonClicked()));
  connect(ui_->allow_looking, SIGNAL(toggled(bool)), this, SLOT(allowLookingToggled(bool)));
  connect(ui_->allow_replanning, SIGNAL(toggled(bool)), this, SLOT(allowReplanningToggled(bool)));
  connect(ui_->allow_external_program, SIGNAL(toggled(bool)), this, SLOT(allowExternalProgramCommunication(bool)));
  connect(ui_->planning_algorithm_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(planningAlgorithmIndexChanged(int)));
  connect(ui_->planning_algorithm_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(planningAlgorithmIndexChanged(int)));
  connect(ui_->import_file_button, SIGNAL(clicked()), this, SLOT(importFileButtonClicked()));
  connect(ui_->import_url_button, SIGNAL(clicked()), this, SLOT(importUrlButtonClicked()));
  connect(ui_->clear_scene_button, SIGNAL(clicked()), this, SLOT(clearSceneButtonClicked()));
  connect(ui_->scene_scale, SIGNAL(valueChanged(int)), this, SLOT(sceneScaleChanged(int)));
  connect(ui_->scene_scale, SIGNAL(sliderPressed()), this, SLOT(sceneScaleStartChange()));
  connect(ui_->scene_scale, SIGNAL(sliderReleased()), this, SLOT(sceneScaleEndChange()));
  connect(ui_->remove_object_button, SIGNAL(clicked()), this, SLOT(removeObjectButtonClicked()));
  connect(ui_->object_x, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_y, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_z, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_rx, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_ry, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_rz, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->publish_current_scene_button, SIGNAL(clicked()), this, SLOT(publishSceneButtonClicked()));
  connect(ui_->collision_objects_list, SIGNAL(itemSelectionChanged()), this, SLOT(selectedCollisionObjectChanged()));
  connect(ui_->collision_objects_list, SIGNAL(itemChanged(QListWidgetItem*)), this,
          SLOT(collisionObjectChanged(QListWidgetItem*)));
  connect(ui_->path_constraints_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(pathConstraintsIndexChanged(int)));
  connect(ui_->clear_octomap_button, SIGNAL(clicked()), this, SLOT(onClearOctomapClicked()));
  connect(ui_->planning_scene_tree, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this,
          SLOT(warehouseItemNameChanged(QTreeWidgetItem*, int)));
  connect(ui_->reset_db_button, SIGNAL(clicked()), this, SLOT(resetDbButtonClicked()));
  connect(ui_->export_scene_text_button, SIGNAL(clicked()), this, SLOT(exportAsTextButtonClicked()));
  connect(ui_->import_scene_text_button, SIGNAL(clicked()), this, SLOT(importFromTextButtonClicked()));
  connect(ui_->load_state_button, SIGNAL(clicked()), this, SLOT(loadStateButtonClicked()));
  connect(ui_->save_start_state_button, SIGNAL(clicked()), this, SLOT(saveStartStateButtonClicked()));
  connect(ui_->save_goal_state_button, SIGNAL(clicked()), this, SLOT(saveGoalStateButtonClicked()));
  connect(ui_->set_as_start_state_button, SIGNAL(clicked()), this, SLOT(setAsStartStateButtonClicked()));
  connect(ui_->set_as_goal_state_button, SIGNAL(clicked()), this, SLOT(setAsGoalStateButtonClicked()));
  connect(ui_->remove_state_button, SIGNAL(clicked()), this, SLOT(removeStateButtonClicked()));
  connect(ui_->clear_states_button, SIGNAL(clicked()), this, SLOT(clearStatesButtonClicked()));
  connect(ui_->approximate_ik, SIGNAL(stateChanged(int)), this, SLOT(approximateIKChanged(int)));

  connect(ui_->detect_objects_button, SIGNAL(clicked()), this, SLOT(detectObjectsButtonClicked()));
  connect(ui_->pick_button, SIGNAL(clicked()), this, SLOT(pickObjectButtonClicked()));
  connect(ui_->place_button, SIGNAL(clicked()), this, SLOT(placeObjectButtonClicked()));
  connect(ui_->detected_objects_list, SIGNAL(itemSelectionChanged()), this, SLOT(selectedDetectedObjectChanged()));
  connect(ui_->detected_objects_list, SIGNAL(itemChanged(QListWidgetItem*)), this,
          SLOT(detectedObjectChanged(QListWidgetItem*)));
  connect(ui_->support_surfaces_list, SIGNAL(itemSelectionChanged()), this, SLOT(selectedSupportSurfaceChanged()));

  connect(ui_->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));

  QShortcut* copy_object_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), ui_->collision_objects_list);
  connect(copy_object_shortcut, SIGNAL(activated()), this, SLOT(copySelectedCollisionObject()));

  ui_->reset_db_button->hide();
  ui_->background_job_progress->hide();
  ui_->background_job_progress->setMaximum(0);

  ui_->tabWidget->setCurrentIndex(0);

  known_collision_objects_version_ = 0;

  planning_scene_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  planning_scene_world_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningSceneWorld>("planning_scene_world", 1);

  object_recognition_client_ = rclcpp_action::create_client<object_recognition_msgs::action::ObjectRecognition>(
        node_->get_node_base_interface(),
        node_->get_node_graph_interface(),
        node_->get_node_logging_interface(),
        node_->get_node_waitables_interface(),
        OBJECT_RECOGNITION_ACTION);

  object_recognition_subscriber_ = node_->create_subscription<object_recognition_msgs::msg::RecognizedObjectArray>(
    "recognized_object_array", 1, std::bind(&MotionPlanningFrame::listenDetectedObjects, this, std::placeholders::_1));

  if (!object_recognition_client_->wait_for_action_server(std::chrono::seconds(3)))
    RCLCPP_ERROR(LOGGER, "Action server: %s not available", OBJECT_RECOGNITION_ACTION.c_str());

  try
  {
    planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface("rviz_planning_scene_interface_node"));
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to load planning_scene_interface_: %s", ex.what());
  }

  try
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
#if 0 // TODO (ddengster): Enable when moveit_ros_perception is ported
    if (ps)
    {
      semantic_world_.reset(new moveit::semantic_world::SemanticWorld(ps));
    }
    else
      semantic_world_.reset();
    if (semantic_world_)
    {
      semantic_world_->addTableCallback(boost::bind(&MotionPlanningFrame::updateTables, this));
    }
#endif
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to get semantic world: %s", ex.what());
  }
}

MotionPlanningFrame::~MotionPlanningFrame()
{
  delete ui_;
}

void MotionPlanningFrame::approximateIKChanged(int state)
{
  planning_display_->useApproximateIK(state == Qt::Checked);
}

void MotionPlanningFrame::setItemSelectionInList(const std::string& item_name, bool selection, QListWidget* list)
{
  QList<QListWidgetItem*> found_items = list->findItems(QString(item_name.c_str()), Qt::MatchExactly);
  for (QListWidgetItem* found_item : found_items)
    found_item->setSelected(selection);
}

void MotionPlanningFrame::allowExternalProgramCommunication(bool enable)
{
  // This is needed to prevent UI event (resuming the options) triggered
  // before getRobotInteraction() is loaded and ready
  if (first_time_)
    return;

  planning_display_->getRobotInteraction()->toggleMoveInteractiveMarkerTopic(enable);
  planning_display_->toggleSelectPlanningGroupSubscription(enable);
  if (enable)
  {
    using std::placeholders::_1;
    plan_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/rviz/moveit/plan", 1, std::bind(&MotionPlanningFrame::remotePlanCallback, this, _1));
    execute_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/rviz/moveit/execute", 1, std::bind(&MotionPlanningFrame::remoteExecuteCallback, this, _1));
    stop_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/rviz/moveit/stop", 1, std::bind(&MotionPlanningFrame::remoteStopCallback, this, _1));
    update_start_state_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/rviz/moveit/update_start_state", 1, std::bind(&MotionPlanningFrame::remoteUpdateStartStateCallback, this, _1));
    update_goal_state_subscriber_=  node_->create_subscription<std_msgs::msg::Empty>(
      "/rviz/moveit/update_goal_state", 1, std::bind(&MotionPlanningFrame::remoteUpdateGoalStateCallback, this, _1));
    update_custom_start_state_subscriber_=  node_->create_subscription<moveit_msgs::msg::RobotState>(
      "/rviz/moveit/update_custom_start_state", 1, std::bind(&MotionPlanningFrame::remoteUpdateCustomStartStateCallback, this, _1));
    update_custom_goal_state_subscriber_=  node_->create_subscription<moveit_msgs::msg::RobotState>(
      "/rviz/moveit/update_custom_goal_state", 1, std::bind(&MotionPlanningFrame::remoteUpdateCustomGoalStateCallback, this, _1));
  }
  else
  {  // disable
    plan_subscriber_.reset();
    execute_subscriber_.reset();
    stop_subscriber_.reset();
    update_start_state_subscriber_.reset();
    update_goal_state_subscriber_.reset();
    update_custom_start_state_subscriber_.reset();
    update_custom_goal_state_subscriber_.reset();
  }
}

void MotionPlanningFrame::fillPlanningGroupOptions()
{
  const QSignalBlocker planning_group_blocker(ui_->planning_group_combo_box);
  ui_->planning_group_combo_box->clear();

  const robot_model::RobotModelConstPtr& kmodel = planning_display_->getRobotModel();
  for (const std::string& group_name : kmodel->getJointModelGroupNames())
    ui_->planning_group_combo_box->addItem(QString::fromStdString(group_name));
}

void MotionPlanningFrame::fillStateSelectionOptions()
{
  const QSignalBlocker start_state_blocker(ui_->start_state_combo_box);
  const QSignalBlocker goal_state_blocker(ui_->goal_state_combo_box);
  ui_->start_state_combo_box->clear();
  ui_->goal_state_combo_box->clear();

  if (!planning_display_->getPlanningSceneMonitor())
    return;

  const robot_model::RobotModelConstPtr& robot_model = planning_display_->getRobotModel();
  std::string group = planning_display_->getCurrentPlanningGroup();
  if (group.empty())
    return;
  const robot_model::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
  if (jmg)
  {
    ui_->start_state_combo_box->addItem(QString("<random valid>"));
    ui_->start_state_combo_box->addItem(QString("<random>"));
    ui_->start_state_combo_box->addItem(QString("<current>"));
    ui_->start_state_combo_box->addItem(QString("<same as goal>"));
    ui_->start_state_combo_box->addItem(QString("<previous>"));

    ui_->goal_state_combo_box->addItem(QString("<random valid>"));
    ui_->goal_state_combo_box->addItem(QString("<random>"));
    ui_->goal_state_combo_box->addItem(QString("<current>"));
    ui_->goal_state_combo_box->addItem(QString("<same as start>"));
    ui_->goal_state_combo_box->addItem(QString("<previous>"));

    const std::vector<std::string>& known_states = jmg->getDefaultStateNames();
    if (!known_states.empty())
    {
      ui_->start_state_combo_box->insertSeparator(ui_->start_state_combo_box->count());
      ui_->goal_state_combo_box->insertSeparator(ui_->goal_state_combo_box->count());
      for (const std::string& known_state : known_states)
      {
        ui_->start_state_combo_box->addItem(QString::fromStdString(known_state));
        ui_->goal_state_combo_box->addItem(QString::fromStdString(known_state));
      }
    }

    ui_->start_state_combo_box->setCurrentIndex(2);  // default to 'current'
    ui_->goal_state_combo_box->setCurrentIndex(2);   // default to 'current'
  }
}

void MotionPlanningFrame::changePlanningGroupHelper()
{
  if (!planning_display_->getPlanningSceneMonitor())
    return;

  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::fillStateSelectionOptions, this));
  planning_display_->addMainLoopJob(
      boost::bind(&MotionPlanningFrame::populateConstraintsList, this, std::vector<std::string>()));

  const robot_model::RobotModelConstPtr& robot_model = planning_display_->getRobotModel();
  std::string group = planning_display_->getCurrentPlanningGroup();
  planning_display_->addMainLoopJob(
      boost::bind(&MotionPlanningParamWidget::setGroupName, ui_->planner_param_treeview, group));
  planning_display_->addMainLoopJob(
      [=]() { ui_->planning_group_combo_box->setCurrentText(QString::fromStdString(group)); });

  if (!group.empty() && robot_model)
  {
    RCLCPP_INFO(LOGGER, "group %s", group.c_str());
    if (move_group_ && move_group_->getName() == group)
      return;
    RCLCPP_INFO(LOGGER, "Constructing new MoveGroup connection for group '%s' in namespace '%s'", group.c_str(),
             planning_display_->getMoveGroupNS().c_str());
    moveit::planning_interface::MoveGroupInterface::Options opt(group);
    opt.robot_model_ = robot_model;
    opt.robot_description_.clear();
    //opt.node_handle_ = ros::NodeHandle(planning_display_->getMoveGroupNS());
    opt.node_ = node_;
    try
    {
#ifdef RVIZ_TF1
      std::shared_ptr<tf2_ros::Buffer> tf_buffer = moveit::planning_interface::getSharedTF();
#else
      //@note: tf2 no longer accessible?
      // /std::shared_ptr<tf2_ros::Buffer> tf_buffer = context_->getFrameManager()->getTF2BufferPtr(); 
      std::shared_ptr<tf2_ros::Buffer> tf_buffer;
#endif
      move_group_.reset(new moveit::planning_interface::MoveGroupInterface(opt, tf_buffer, rclcpp::Duration(30)));
#if 0 // TODO (ddengster): Enable when moveit_ros_warehouse is ported
      if (planning_scene_storage_)
        move_group_->setConstraintsDatabase(ui_->database_host->text().toStdString(), ui_->database_port->value());
#endif
    }
    catch (std::exception& ex)
    {
      RCLCPP_ERROR(LOGGER, "%s", ex.what());
    }
    planning_display_->addMainLoopJob(
        boost::bind(&MotionPlanningParamWidget::setMoveGroup, ui_->planner_param_treeview, move_group_));
    if (move_group_)
    {
      move_group_->allowLooking(ui_->allow_looking->isChecked());
      move_group_->allowReplanning(ui_->allow_replanning->isChecked());
      bool has_unique_endeffector = !move_group_->getEndEffectorLink().empty();
      planning_display_->addMainLoopJob([=]() { ui_->use_cartesian_path->setEnabled(has_unique_endeffector); });
      moveit_msgs::msg::PlannerInterfaceDescription desc;
      if (move_group_->getInterfaceDescription(desc))
      {
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlannersList, this, desc));
      }
      planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this),
                                          "populateConstraintsList");

      if (first_time_)
      {
        first_time_ = false;
        const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
        if (ps)
        {
          planning_display_->setQueryStartState(ps->getCurrentState());
          planning_display_->setQueryGoalState(ps->getCurrentState());
        }
        // This ensures saved UI settings applied after planning_display_ is ready
        planning_display_->useApproximateIK(ui_->approximate_ik->isChecked());
        if (ui_->allow_external_program->isChecked())
          planning_display_->addMainLoopJob(
              boost::bind(&MotionPlanningFrame::allowExternalProgramCommunication, this, true));
      }
    }
  }
}

void MotionPlanningFrame::changePlanningGroup()
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::changePlanningGroupHelper, this),
                                      "Frame::changePlanningGroup");
  joints_tab_->changePlanningGroup(planning_display_->getCurrentPlanningGroup(),
                                   planning_display_->getQueryStartStateHandler(),
                                   planning_display_->getQueryGoalStateHandler());
}

void MotionPlanningFrame::sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  if (update_type & planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY)
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
}

void MotionPlanningFrame::importResource(const std::string& path)
{
  if (planning_display_->getPlanningSceneMonitor())
  {
    shapes::Mesh* mesh = shapes::createMeshFromResource(path);
    if (mesh)
    {
      std::size_t slash = path.find_last_of("/\\");
      std::string name = path.substr(slash + 1);
      shapes::ShapeConstPtr shape(mesh);
      Eigen::Isometry3d pose;
      pose.setIdentity();

      if (planning_display_->getPlanningSceneRO()->getCurrentState().hasAttachedBody(name))
      {
        QMessageBox::warning(this, QString("Duplicate names"), QString("An attached object named '")
                                                                   .append(name.c_str())
                                                                   .append("' already exists. Please rename the "
                                                                           "attached object before importing."));
        return;
      }

      // If the object already exists, ask the user whether to overwrite or rename
      if (planning_display_->getPlanningSceneRO()->getWorld()->hasObject(name))
      {
        QMessageBox msg_box;
        msg_box.setText("There exists another object with the same name.");
        msg_box.setInformativeText("Would you like to overwrite it?");
        msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
        msg_box.setDefaultButton(QMessageBox::No);
        int ret = msg_box.exec();

        switch (ret)
        {
          case QMessageBox::Yes:
            // Overwrite was clicked
            {
              planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
              if (ps)
              {
                ps->getWorldNonConst()->removeObject(name);
                addObject(ps->getWorldNonConst(), name, shape, pose);
              }
            }
            break;
          case QMessageBox::No:
          {
            // Don't overwrite was clicked. Ask for another name
            bool ok = false;
            QString text = QInputDialog::getText(
                this, tr("Choose a new name"), tr("Import the new object under the name:"), QLineEdit::Normal,
                QString::fromStdString(name + "-" + std::to_string(planning_display_->getPlanningSceneRO()->getWorld()->size())),
                &ok);
            if (ok)
            {
              if (!text.isEmpty())
              {
                name = text.toStdString();
                planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
                if (ps)
                {
                  if (ps->getWorld()->hasObject(name))
                    QMessageBox::warning(
                        this, "Name already exists",
                        QString("The name '").append(name.c_str()).append("' already exists. Not importing object."));
                  else
                    addObject(ps->getWorldNonConst(), name, shape, pose);
                }
              }
              else
                QMessageBox::warning(this, "Object not imported", "Cannot use an empty name for an imported object");
            }
            break;
          }
          default:
            // Pressed cancel, do nothing
            break;
        }
      }
      else
      {
        planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
        if (ps)
          addObject(ps->getWorldNonConst(), name, shape, pose);
      }
    }
    else
    {
      QMessageBox::warning(this, QString("Import error"), QString("Unable to import scene"));
      return;
    }
  }
}

void MotionPlanningFrame::enable()
{
  ui_->planning_algorithm_combo_box->clear();
  ui_->library_label->setText("NO PLANNING LIBRARY LOADED");
  ui_->library_label->setStyleSheet("QLabel { color : red; font: bold }");
  ui_->object_status->setText("");

  // activate the frame
  parentWidget()->show();
}

void MotionPlanningFrame::disable()
{
  move_group_.reset();
  parentWidget()->hide();
}

void MotionPlanningFrame::tabChanged(int index)
{
  if (scene_marker_ && ui_->tabWidget->tabText(index).toStdString() != TAB_OBJECTS)
    scene_marker_.reset();
  else if (ui_->tabWidget->tabText(index).toStdString() == TAB_OBJECTS)
    selectedCollisionObjectChanged();
}

void MotionPlanningFrame::updateSceneMarkers(float /*wall_dt*/, float /*ros_dt*/)
{
  if (scene_marker_)
    scene_marker_->update();
}

void MotionPlanningFrame::updateExternalCommunication()
{
  if (ui_->allow_external_program->isChecked())
  {
    planning_display_->getRobotInteraction()->toggleMoveInteractiveMarkerTopic(true);
  }
}

}  // namespace moveit_rviz_plugin
