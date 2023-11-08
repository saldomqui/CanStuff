/************************************************************************
 * by Salvador Dominguez
 *
 ************************************************************************/

// The header of the class
#include "can_sniffer.h"

// Some constants
#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180.0)
#endif

#ifndef RAD2DEG
#define RAD2DEG (180.0 / M_PI)
#endif

#ifndef M_PI_TWO
#define M_PI_TWO (M_PI * 2.0)
#endif

using namespace std;

map<unsigned int, can_data> all_id_list;
vector<unsigned int> can_excluded;

struct compFunction
{
    template <typename T>
    bool operator()(const T &elem1, const T &elem2) const
    {
        return elem1.second.freq >= elem2.second.freq;
    }
};

bool variableFound(int i)
{
    return ((i % 2) == 1);
}

// CanSniffer class constructor
// Parameters reading, subscriptions, publications, control objects creation, threads creation, variables initialitation
CanSniffer::CanSniffer() : local_nh_("~"),
                           global_nh_(""),
                           selectedMsgId(0),
                           sel_msg_freq(0),
                           sel_msg_freq_change(0)
{
    string fileNameInspectdVariables;
    string fileNameCanFilteredMsgs;
    bool extendedCanId = false;

    local_nh_.getParam("extendedCanId", extendedCanId);
    cout << "can_sniffer: extendedCanId" << int(extendedCanId) << endl;

    // Load local parameters
    local_nh_.getParam("fileNameCanFilteredMsgs", fileNameCanFilteredMsgs); // Can msgs filtered file
    cout << "can_sniffer: fileNameCanFilteredMsgs:" << fileNameCanFilteredMsgs << endl;
    loadCanMsgsFilteredFromXML(fileNameCanFilteredMsgs);

    local_nh_.getParam("fileNameInspectedVariables", fileNameInspectedVariables);
    cout << "can_sniffer: fileNameInspectedVariables:" << fileNameInspectedVariables << endl;
    loadVarListFromXML(fileNameInspectedVariables);

    // Subscribers
    ROS_INFO("can_sniffer: Subscribing to topics\n");

    can_sub = global_nh_.subscribe<thorvald_base::CANFrame>("/can/data", 33, &CanSniffer::canCallback, this);

    on_off_msg_sub = global_nh_.subscribe<can_sniffer::OnOffMsg>("/can/on_off_msg", 1, &CanSniffer::onOffMsgCallback, this);
    select_msg_sub = global_nh_.subscribe<std_msgs::Int32>("/can/select_msg", 1, &CanSniffer::selectMsgCallback, this);
    inspected_variable_sub = global_nh_.subscribe<can_sniffer::CanVariableData>("/can/inspected_variable", 1, &CanSniffer::inspectedVariableCallback, this);

    // Service servers
    get_var_list_service = global_nh_.advertiseService("/can/get_var_list", &CanSniffer::getVarListCallback, this);

    // Publishers
    pub_can_all = global_nh_.advertise<can_sniffer::CanMsgIdList>("/can/all", 1, this);         // Data of all controllers in the same topic
    pub_can_changed = global_nh_.advertise<can_sniffer::CanMsgIdList>("/can/changed", 1, this); // Data of all controllers in the same topic
    can_pub = global_nh_.advertise<thorvald_base::CANFrame>("/can/data_to_send", 33, this);
    selected_msg_pub = global_nh_.advertise<can_sniffer::CanMsgId>("/can/selected_msg", 1, this);
}

// Destructor
CanSniffer::~CanSniffer()
{
}

bool CanSniffer::getVarListCallback(can_sniffer::GetVarList::Request &req, can_sniffer::GetVarList::Response &res)
{
    cout << "can_sniffer: requesting variables list" << endl;

    for (map<string, per_id_map_>::iterator it_var = inspect_var_list.begin(); it_var != inspect_var_list.end(); it_var++)
    {
        can_sniffer::VarData vd;

        vd.name = it_var->first;

        for (per_id_map_::iterator it_id = it_var->second.begin(); it_id != it_var->second.end(); it_id++)
        {
            can_sniffer::CanMsgVariable mv;

            mv.msg_id = it_id->first;
            mv.offset = it_id->second.offset;
            mv.scale = it_id->second.scale;

            // cout << "can_sniffer: var:" << vd.name << " msg_id:" << mv.msg_id << " offset:" << mv.offset << " scale:" << mv.scale << endl;
            for (vector<bit_select>::iterator it_bits = it_id->second.bit_masks.begin(); it_bits != it_id->second.bit_masks.end(); it_bits++)
            {
                can_sniffer::BitMaskData md;

                md.byte_num = it_bits->byte_num;
                md.mask = it_bits->mask;
                mv.bit_masks.push_back(md);
                // cout << "can_sniffer: byte_idx:" << int(md.byte_num) << " mask:" << int(md.mask) << endl;
            }
            vd.data.push_back(mv);
        }
        res.variables.push_back(vd);
    }

    return true;
}

vector<string> CanSniffer::getVarList(unsigned int id)
{
    vector<string> var_list;

    // Search in all variables
    for (map<string, per_id_map_>::iterator it_var = inspect_var_list.begin(); it_var != inspect_var_list.end(); it_var++)
    {
        // cout << "can_sniffer: var:" << it_var->first << " searching id:" << std::hex << id << " num_ids:" << it_var->second.size() << endl;
        per_id_map_::iterator it_id = it_var->second.find(id);

        if (it_id != it_var->second.end()) // Found id in list of id of this variable
        {
            // cout << "can_sniffer: var:" << it_var->first << " is in msg id:" << id << endl;
            var_list.push_back(it_var->first);
        }
    }
    return var_list;
}

void CanSniffer::inspectedVariableCallback(can_sniffer::CanVariableData msg)
{
    can_variable_data rcv_data;

    // cout << "can_sniffer: Inspected variable data comming" << endl;

    // fill up data structure with received data
    for (vector<can_sniffer::BitMaskData>::iterator it_mask = msg.bit_masks.begin(); it_mask != msg.bit_masks.end(); it_mask++)
    {
        bit_select bs;

        bs.byte_num = static_cast<unsigned char>(it_mask->byte_num);
        bs.mask = static_cast<unsigned char>(it_mask->mask);
        rcv_data.bit_masks.push_back(bs);
    }

    rcv_data.freq = static_cast<unsigned int>(msg.freq);
    rcv_data.offset = static_cast<float>(msg.offset);
    rcv_data.scale = static_cast<float>(msg.scale);
    rcv_data.comment = msg.comment;

    map<string, per_id_map_>::iterator it_var = inspect_var_list.find(msg.name);

    if (it_var == inspect_var_list.end())
    {
        // cout << "can_sniffer: variable:" << msg.name << " is not in the list. Adding..." << endl;

        map<unsigned int, can_variable_data> per_id_data;

        per_id_data.insert(make_pair(static_cast<unsigned int>(msg.msg_id), rcv_data));

        inspect_var_list.insert(make_pair(msg.name, per_id_data));
    }
    else // Found variable in current list. Update it
    {
        // cout << "can_sniffer: viariable:" << msg.name << " is already in the list. Updating..." << endl;

        // Search id in variable's id list
        per_id_map_::iterator it_id = it_var->second.find(msg.msg_id);

        if (it_id != it_var->second.end()) // Fond ID in id list of this variable. update data
        {
            // cout << "id:" << msg.msg_id << " is already in the id list of variable:" << msg.name << ". Updating..." << endl;
            it_id->second = rcv_data;
        }
        else
        {
            // cout << "id:" << msg.msg_id << " is not in the id list of variable:" << msg.name << ". Adding..." << endl;
            it_var->second.insert(make_pair(static_cast<unsigned int>(msg.msg_id), rcv_data));
        }
    }
    saveInspectedVarDataToXMLFile(fileNameInspectedVariables);
}

void CanSniffer::selectMsgCallback(std_msgs::Int32 msg)
{
    cout << "can_sniffer: selected msg id:" << int(msg.data) << endl;
    selectedMsgId = msg.data;
}

void CanSniffer::onOffMsgCallback(can_sniffer::OnOffMsg msg)
{
    // cout << "can_sniffer: msg:" << msg.id << " active:" << int(msg.active) << endl;
    std::map<unsigned int, can_data>::iterator it_msg = all_id_list.find(msg.id);

    if (it_msg != all_id_list.end())
    {
        // cout << "found id:" << msg.id << " in current msgs list:" << " is currently active?:" << int(it_msg->second.active) << endl;
        it_msg->second.active = msg.active;
    }
}


// Callback that receives the CAN ROS topics from can_reader and decodes them to obtain relevant information about the status of the car
void CanSniffer::canCallback(const thorvald_base::CANFrameConstPtr &msg_can)
{
    ros::Time t_now = ros::Time::now();
    static ros::Time tstamp_prev = t_now;
    can_sniffer::CanMsgId msg_id;
    static int prev_sel_msg_count = 0;
    static int prev_sel_msg_changed_count = 0;
    static ros::Time tstamp_sel_msg_prev(t_now);

    // cout << "can_sniffer: Normal ID Can data comming" << endl;

    // cout << "can_sniffer:  can data comming..." << endl;
    /*
        cout  << "ID:" << hex << msg_can->id << "[";

        for (unsigned short i=0;i<msg_can->length;i++)
        {
            cout << hex  << " " << int(msg_can->data[i]);
        }
        cout << "]" << endl;
*/

    //-------------------- BUILD LISTS OF ALL MSGS RECEIVED SO FAR AND ALSO THOSE WHOSE VALUES HAVE CHANGED SINCE PREVIOUS TIME RECEIVED -------

    map<unsigned int, can_data>::iterator it = all_id_list.find(static_cast<unsigned int>(msg_can->id));

    if (can_excluded.size())
    {
        if (find(can_excluded.begin(), can_excluded.end(), msg_can->id) == can_excluded.end()) // Is not in the excluded list
        {
            // cout  << "can_sniffer: PUBLISHING CAN MSG ID:" << hex << msg_can->id << endl;
            can_pub.publish(*msg_can);
        }
    }
    if (it == all_id_list.end()) // The incomming mesg hasn't been received previously. Add it to the list of total and changed
    {
        // cout << "can_sniffer: new msgs, adding it to the list" << endl;
        can_data cand;

        cand.tstamp = t_now;
        cand.msg_can.id = static_cast<unsigned int>(msg_can->id);
        // cout << "can_sniffer: dlc:" << msg_can.dlc << endl;
        cand.msg_can.length = msg_can->length;
        for (int i = 0; i < msg_can->length; i++)
        {
            cand.msg_can.data.push_back(msg_can->data[i]);
        }
        cand.count = 1;
        cand.changed_cnt = 1;
        cand.changed = true;
        cand.freq = 1;
        cand.active = true;

        // cout << "can_sniffer: inserting" << endl;
        all_id_list.insert(make_pair(cand.msg_can.id, cand));
        // cout << "can_sniffer: inserted" << endl;
    }
    else // The incomming msg already existed in the previous received msgs
    {
        // cout << "can_sniffer: known msgs, updating data" << endl;

        it->second.changed = false;
        it->second.count++; // Increment the counter
        it->second.freq++;  // Frequency counter
        it->second.tstamp = t_now;
        it->second.msg_can.length = msg_can->length;

        for (int i = 0; i < it->second.msg_can.length; i++)
        {
            if (it->second.msg_can.data[i] != msg_can->data[i])
            {
                it->second.changed = true;
            }
            it->second.msg_can.data[i] = msg_can->data[i];
        }

        if (it->second.changed)
        {
            it->second.changed_cnt++;
            it->second.freq_change++; // Frequency change counter
        }

        if (msg_can->id == selectedMsgId)
        {
            // cout << "can_sniffer: ord_list... element:" << it_set->second.msg_can.id << endl;
            msg_id.stamp = it->second.tstamp;
            msg_id.id = it->second.msg_can.id;
            msg_id.count = it->second.count;
            msg_id.count_changed = it->second.changed_cnt;

            if ((t_now - tstamp_sel_msg_prev).toSec() > 1.0)
            {
                sel_msg_freq = it->second.count - prev_sel_msg_count;
                sel_msg_freq_change = it->second.changed_cnt - prev_sel_msg_changed_count;
                tstamp_sel_msg_prev = t_now;
                prev_sel_msg_count = it->second.count;
                prev_sel_msg_changed_count = it->second.changed_cnt;
            }
            msg_id.freq_change = sel_msg_freq_change;
            msg_id.freq = sel_msg_freq;
            msg_id.dlc = it->second.msg_can.length;
            msg_id.active = it->second.active;
            msg_id.data.clear();
            for (int i = 0; i < msg_id.dlc; i++)
                msg_id.data.push_back(it->second.msg_can.data[i]);

            selected_msg_pub.publish(msg_id);
        }
    }

    if ((t_now - tstamp_prev).toSec() > 1.0)
    {
        can_sniffer::CanMsgIdList msg_all;
        can_sniffer::CanMsgIdList msg_changed;

        std::set<std::pair<unsigned int, can_data>, compFunction> all_id_ordered_list;

        for (it = all_id_list.begin(); it != all_id_list.end(); it++)
        {
            all_id_ordered_list.insert(make_pair(it->first, it->second));
            it->second.freq = 0; // Reset frequency counter
            it->second.freq_change = 0;
            it->second.changed = false;
        }

        for (std::set<per_id_can_data_>::iterator it_set = all_id_ordered_list.begin(); it_set != all_id_ordered_list.end(); it_set++)
        {
            // cout << "can_sniffer: ord_list... element:" << it_set->second.msg_can.id << endl;
            msg_id.stamp = it_set->second.tstamp;
            msg_id.id = it_set->second.msg_can.id;
            msg_id.count = it_set->second.count;
            msg_id.count_changed = it_set->second.changed_cnt;
            msg_id.freq_change = it_set->second.freq_change;
            msg_id.freq = it_set->second.freq;
            msg_id.dlc = it_set->second.msg_can.length;
            msg_id.active = it_set->second.active;
            msg_id.variables = getVarList(msg_id.id);

            msg_id.data.clear();
            for (int i = 0; i < msg_id.dlc; i++)
                msg_id.data.push_back(it_set->second.msg_can.data[i]);

            msg_all.id_list.push_back(msg_id);

            if (it_set->second.changed)
            {
                bool found = false;
                if (can_excluded.size())
                {
                    if (find(can_excluded.begin(), can_excluded.end(), msg_id.id) == can_excluded.end()) // Is not in the excluded list
                    {
                    }
                    else
                        found = true;
                }
                if (!found)
                    msg_changed.id_list.push_back(msg_id);
            }
        }

        pub_can_all.publish(msg_all);
        if (msg_changed.id_list.size())
            pub_can_changed.publish(msg_changed);

        tstamp_prev = t_now;
    }

    //-----------------------------------------------------------------------------------------------------------------------------------------------------
}

// Load the msgs that pass the filter
bool CanSniffer::loadCanMsgsFilteredFromXML(string fileName)
{
    xmlDoc *doc = NULL;
    xmlNode *root_element = NULL;

    cout << "can_sniffer: Msgs filter file:" << fileName.c_str() << endl;
    doc = xmlReadFile(fileName.c_str(), NULL, 0);
    if (doc != NULL)
    {
        /*Get the root element node */
        root_element = xmlDocGetRootElement(doc);

        if (root_element->type == XML_ELEMENT_NODE)
        {
            if (!xmlStrcmp(root_element->name, xmlCharStrdup("can_msg")))
            {
                xmlNodePtr cNode;

                cNode = root_element->xmlChildrenNode;

                can_excluded.clear();

                while (cNode != NULL)
                {
                    if (!xmlStrcmp(cNode->name, xmlCharStrdup("msg")))
                    {
                        xmlChar *prop;
                        unsigned int id;

                        prop = xmlGetProp(cNode, xmlCharStrdup("id"));
                        id = static_cast<unsigned int>(atoi(reinterpret_cast<char *>(prop)));
                        xmlFree(prop);

                        cout << "can_sniffer: CAN filtered out msg id:" << id << endl;
                        can_excluded.push_back(id);
                    }
                    cNode = cNode->next;
                }
                cout << "can_sniffer: -------------------------- END CAN MSGS FILTER  -------------------------" << endl;

                cout << "can_sniffer: Loaded:" << can_excluded.size() << " msg ids" << endl;
            }
        }

        /*free the document */
        xmlFreeDoc(doc);

        /*
         *Free the global variables that may
         *have been allocated by the parser.
         */
        xmlCleanupParser();
        return true;
    }
    else
    {
        return false;
    }
}

bool CanSniffer::saveInspectedVarDataToXMLFile(const string &fileName)
{
    // Generate XML file with submap path information
    std::ofstream resultOutput;

    resultOutput.open(fileName.c_str(), std::ofstream::out | std::ofstream::trunc);

    if (resultOutput.fail())
    {
        std::cout << "can_sniffer:  ERROR!! Unable to open: " << fileName << " file!" << std::endl;
        return false;
    }
    else
    {
        resultOutput << std::setprecision(6);

        resultOutput << "<?xml version=\"1.0\"?>" << endl;

        resultOutput << "<var_list>" << endl;

        for (map<string, per_id_map_>::iterator it_var = inspect_var_list.begin(); it_var != inspect_var_list.end(); it_var++)
        {
            resultOutput << "  <var ";
            resultOutput << " name=\"";
            resultOutput << it_var->first << "\"";
            resultOutput << ">" << endl;

            for (per_id_map_::iterator it_id = it_var->second.begin(); it_id != it_var->second.end(); it_id++)
            {
                resultOutput << "    <msg_id_hex ";
                resultOutput << " val=\"";
                resultOutput << std::hex << uppercase << std::setfill('0') << std::setw(4) << it_id->first << "\"";
                resultOutput << " freq=\"";
                resultOutput << std::dec << it_id->second.freq << "\"";
                resultOutput << " offset=\"";
                resultOutput << it_id->second.offset << "\"";
                resultOutput << " scale=\"";
                resultOutput << it_id->second.scale << "\"";
                resultOutput << " comment=\"";
                resultOutput << it_id->second.comment << "\"";
                resultOutput << ">" << endl;

                for (vector<bit_select>::iterator it_bits = it_id->second.bit_masks.begin(); it_bits != it_id->second.bit_masks.end(); it_bits++)
                {
                    resultOutput << "      <mask ";
                    resultOutput << " byte_idx=\"";
                    resultOutput << int(it_bits->byte_num) << "\"";
                    resultOutput << " mask_hex=\"";
                    resultOutput << std::hex << uppercase << std::setfill('0') << std::setw(2) << int(it_bits->mask) << "\"" << std::dec;
                    resultOutput << "/>" << endl;
                }
                resultOutput << "    </msg_id_hex>" << endl;
            }
            resultOutput << "  </var>" << endl;
        }

        resultOutput << "</var_list>" << endl;
        resultOutput.close();

    } // END XML FILE GENERATION
    return true;
}

// Load all parameters for available cars from XML file
bool CanSniffer::loadVarListFromXML(const string &fileName)
{
    xmlDoc *doc = NULL;
    xmlNode *root_element = NULL;
    xmlChar *prop;

    cout << "can_sniffer: variables fileName:" << fileName << endl;
    doc = xmlReadFile(fileName.c_str(), NULL, 0);
    if (doc != NULL)
    {
        xmlNodePtr varNode;
        xmlChar *prop;

        /*Get the root element node */
        root_element = xmlDocGetRootElement(doc);

        if (root_element->type == XML_ELEMENT_NODE)
        {
            if (!xmlStrcmp(root_element->name, xmlCharStrdup("var_list")))
            {
                varNode = root_element->xmlChildrenNode;

                while (varNode != NULL)
                {
                    // cout << "can_sniffer:" << varNode->name << endl;
                    if (!xmlStrcmp(varNode->name, xmlCharStrdup("var")))
                    {
                        xmlNodePtr idNode;
                        string var_name;
                        per_id_map_ pim;

                        prop = xmlGetProp(varNode, xmlCharStrdup("name"));
                        var_name.assign(reinterpret_cast<char *>(prop));
                        // cout << "can_sniffer: var_name:" + var_name << endl;

                        xmlFree(prop);

                        idNode = varNode->xmlChildrenNode;

                        while (idNode != NULL) // For every car in the list of available cars
                        {
                            if (!xmlStrcmp(idNode->name, xmlCharStrdup("msg_id_hex")))
                            {
                                xmlNodePtr maskNode;
                                can_variable_data var_data;
                                unsigned int msg_id;
                                std::stringstream ss_id;

                                prop = xmlGetProp(idNode, xmlCharStrdup("val"));

                                ss_id.str(std::string());
                                ss_id << std::hex << string(reinterpret_cast<char *>(prop));
                                ss_id >> msg_id;
                                xmlFree(prop);

                                prop = xmlGetProp(idNode, xmlCharStrdup("freq"));
                                var_data.freq = static_cast<unsigned int>(atoi(reinterpret_cast<char *>(prop)));
                                xmlFree(prop);

                                prop = xmlGetProp(idNode, xmlCharStrdup("offset"));
                                var_data.offset = static_cast<float>(atof(reinterpret_cast<char *>(prop)));
                                xmlFree(prop);

                                prop = xmlGetProp(idNode, xmlCharStrdup("scale"));
                                var_data.scale = static_cast<float>(atof(reinterpret_cast<char *>(prop)));
                                xmlFree(prop);

                                prop = xmlGetProp(idNode, xmlCharStrdup("comment"));
                                var_data.comment.assign(reinterpret_cast<char *>(prop));
                                xmlFree(prop);

                                cout << "can_sniffer: can_id:" << msg_id << endl;

                                maskNode = idNode->xmlChildrenNode;

                                while (maskNode != NULL) // For every car in the list of available cars
                                {
                                    if (!xmlStrcmp(maskNode->name, xmlCharStrdup("mask")))
                                    {
                                        bit_select bs;
                                        unsigned int aux;
                                        std::stringstream ss;

                                        prop = xmlGetProp(maskNode, xmlCharStrdup("byte_idx"));
                                        bs.byte_num = static_cast<unsigned char>(atoi(reinterpret_cast<char *>(prop)));
                                        xmlFree(prop);

                                        prop = xmlGetProp(maskNode, xmlCharStrdup("mask_hex"));
                                        ss.str(std::string());
                                        ss << std::hex;
                                        ss << string(reinterpret_cast<char *>(prop));
                                        ss >> aux;

                                        cout << "can_sniffer: mask aux:" << aux << " string:" << string(reinterpret_cast<char *>(prop)) << endl;
                                        bs.mask = static_cast<unsigned char>(aux);
                                        xmlFree(prop);

                                        var_data.bit_masks.push_back(bs);

                                        cout << "can_sniffer: byte_idx:" << int(bs.byte_num) << " mask:" << int(bs.mask) << endl;
                                    }
                                    maskNode = maskNode->next;
                                }
                                pim.insert(make_pair(msg_id, var_data));
                            }
                            idNode = idNode->next;
                        }
                        inspect_var_list.insert(make_pair(var_name, pim));
                    }
                    varNode = varNode->next;
                }
            }
        }

        /*free the document */
        xmlFreeDoc(doc);
        xmlCleanupParser();
        return true;
    }
    else
    {
        return false;
    }
}
