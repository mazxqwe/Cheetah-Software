//#include "../include/JoystickTest.h"
//#include "ui_JoystickTest.h"
//#include <QTimer>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int ne_main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");


  ros::NodeHandle n;

  
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}




/*
JoystickTestWindow::JoystickTestWindow(GameController& gamepad, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JoystickTestWindow),
    _gamepad(gamepad)
{
    ui->setupUi(this);

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  timer->start(1000 / 30);
}

JoystickTestWindow::~JoystickTestWindow()
{
    delete ui;
}


void JoystickTestWindow::update() {
  _gamepad.updateGamepadCommand(_command);
  char buffer[256];

  sprintf(buffer, "Left X: %4.2f\n", _command.leftStickAnalog[0]);
  ui->leftXLabel->setText(buffer);

  sprintf(buffer, "Left Y: %4.2f\n", _command.leftStickAnalog[1]);
  ui->leftYLabel->setText(buffer);

  sprintf(buffer, "Right X: %4.2f\n", _command.rightStickAnalog[0]);
  ui->rightXLabel->setText(buffer);

  sprintf(buffer, "Right Y: %4.2f\n", _command.rightStickAnalog[1]);
  ui->rightYLabel->setText(buffer);
}

*/
