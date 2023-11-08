#ifndef CAN_SNIFFER_H
#define CAN_SNIFFER_H

#include <map>
#include <set>
#include <bitset>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

//The XML parser to parse and generate XML files
#include <libxml/parser.h>
#include <libxml/tree.h>

//ROS
#include <ros/ros.h>

#include <std_msgs/Int32.h>

//Can mgs lists
#include "can_sniffer/CanMsgId.h"
#include "can_sniffer/CanMsgIdList.h"
#include "can_sniffer/OnOffMsg.h"
#include "can_sniffer/CanVariableData.h"
#include "can_sniffer/GetVarList.h"
#include "can_sniffer/VarData.h"

#include "thorvald_base/CANFrame.h" //To receive the raw CAN topics from the effiros project

//Namespaces
using namespace std;

typedef struct bit_select_
{
  unsigned char byte_num;
  unsigned char mask;
} bit_select;

typedef struct can_variable_data_
{
  vector<bit_select> bit_masks;
  unsigned int freq;
  float offset;
  float scale;
  string comment;
} can_variable_data;

typedef struct can_data_
{
  ros::Time tstamp;
  thorvald_base::CANFrame msg_can;
  unsigned long long count;
  bool changed;
  unsigned long long changed_cnt;
  unsigned int freq_change;
  unsigned int freq;
  bool active;
} can_data;

typedef map<unsigned int, can_variable_data> per_id_map_;
typedef std::pair<unsigned int, can_data> per_id_can_data_;

/**
* The CanSniffer class
*/
class CanSniffer
{
public:
  /** \fn CanSniffer()
      * \brief CanSniffer class constructor
      */
  CanSniffer();

  /** \fn ~CanSniffer()
      * \brief CanSniffer class destructor
      */
  ~CanSniffer();

private:
  //Ros handles
  ros::NodeHandle global_nh_; //Global ROS Handler
  ros::NodeHandle local_nh_;  //Local ROS Handler

  //Topic publishers
  ros::Publisher pub_can_all;
  ros::Publisher pub_can_changed;
  ros::Publisher can_pub;
  ros::Publisher selected_msg_pub;
  ros::Publisher car_id_pub;

  //Subscribers
  ros::Subscriber can_sub;        //Subscriber to get the CAN data
  ros::Subscriber on_off_msg_sub; //Subscriber to activate/deactivate specific msgs
  ros::Subscriber select_msg_sub;
  ros::Subscriber inspected_variable_sub;

  //Service servers
  ros::ServiceServer get_var_list_service;

  int selectedMsgId;
  int sel_msg_freq;
  int sel_msg_freq_change;
  string fileNameInspectedVariables;

  map<string, per_id_map_> inspect_var_list;

  
  //Topics callbacks
  void canCallback(const thorvald_base::CANFrameConstPtr &msg_can);
  bool loadCanMsgsFilteredFromXML(string fileName);
  void onOffMsgCallback(can_sniffer::OnOffMsg msg);
  void selectMsgCallback(std_msgs::Int32 msg);
  void inspectedVariableCallback(can_sniffer::CanVariableData msg);
  bool getVarListCallback(can_sniffer::GetVarList::Request &req, can_sniffer::GetVarList::Response &res);

  bool saveInspectedVarDataToXMLFile(const string &fileName);
  bool loadVarListFromXML(const string &fileName);
  vector<string> getVarList(unsigned int id);
};

#endif // CAN_SNIFFER_H
