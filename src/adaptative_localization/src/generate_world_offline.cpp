#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Pose3.hh>
#include <array>
#include <ros/ros.h>
#include <XmlRpcValue.h>

std::vector<std::vector<double>> getMatrixParam(ros::NodeHandle &nh, const std::string &key) {
  XmlRpc::XmlRpcValue maze_raw;
  std::vector<std::vector<double>> maze_full;
  if (!nh.getParam(key, maze_raw))
  {
    ROS_ERROR("Failed to get param");
    return maze_full;
  }

  if (maze_raw.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Param is not an array");
    return maze_full;
  }

  for (int i = 0; i < maze_raw.size(); ++i)
  {
    if (maze_raw[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Param[%d] is not an array", i);
      return maze_full;
    }

    std::vector<double> row;
    for (int j = 0; j < maze_raw[i].size(); ++j)
    {
      if (maze_raw[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt){
        row.push_back(static_cast<int>(maze_raw[i][j]));
      }
      else if (maze_raw[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble){
        row.push_back(static_cast<double>(maze_raw[i][j]));
      }
      else
        ROS_WARN("Param[%d][%d] tem tipo inesperado", i, j);
    }
    maze_full.push_back(row);
  }
  return  maze_full;
}

std::vector<std::string> getModelTypes(ros::NodeHandle &nh, const std::string &key) {
  XmlRpc::XmlRpcValue types_param;
  std::vector<std::string> types;
  if (nh.getParam(key, types_param)) {
    for (int i = 0; i < types_param.size(); i++) {
      std::string val = static_cast<std::string>(types_param[i]["type"]);
      types.push_back(val);
    }
  }
  else{
    ROS_ERROR("Failed to get param");
  }
  return types;
}

std::string setModel(const std::string &type, ignition::math::Vector3d pose, ignition::math::Vector3d ori){
static long int index = 0;

// Cria o modelo SDF da caixa como string
std::ostringstream newModelStr;
newModelStr << "<sdf version='1.6'>"
    << "  <model name='"<<type<<"_"<<index<<"'>"
    << "    <pose>"<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "<<ori[0]<<" "<<ori[1]<<" "<<ori[2]<<"</pose>"
    <<"     <include>"
    <<"        <uri>model://"<<type<<"</uri>"
    <<"     </include>"
    << "  </model>"
    << "</sdf>";
index++;
return newModelStr.str();
}


std::string setGeometry(const std::string &type, const std::string &color, ignition::math::Vector3d pose,
            ignition::math::Vector3d ori, ignition::math::Vector3d size){
  static long int index = 0;

  // Cria o modelo SDF da caixa como string
  std::ostringstream newModelStr;
  newModelStr << "<sdf version='1.6'>"
              << "  <model name='"<<type<<"_"<<index<<"'>"
              << "    <static>1</static>"
              << "    <pose>"<<pose[0]<<" "<<pose[1]<<" "<<pose[3]<<" "<<ori[0]<<" "<<ori[1]<<" "<<ori[3]<<"</pose>"
              << "    <link name='link'>"
              << "      <collision name='collision'>"
              << "        <geometry>"
              << "           <"<<type<<">"
              << "           "<<((type == "plane") ? "<normal>0 0 1</normal>" : "")
              << "            <size>"<<size[0]<<" "<<size[1]<<" "<<((type == "plane") ? "" : std::to_string(size[3]))<<"</size>"
              << "          </"<<type<<">"
              << "       </geometry>"
              << "      </collision>"
              << "      <visual name='visual'>"
              << "        <geometry>"
              << "           "<<((type == "plane") ? "<normal>0 0 1</normal>" : "")
              << "           <"<<type<<">"
              << "            <size>"<<size[0]<<" "<<size[1]<<" "<<((type == "plane") ? "" : std::to_string(size[3]))<<"</size>"
              << "          </"<<type<<">"
              << "       </geometry>"
              << "       <material>"
              << "          <script>"
              << "            <uri>file://media/materials/scripts/gazebo.material</uri>"
              << "            <name>Gazebo/"<<color<<"</name>"
              << "          </script>"
              << "      </material>"
              << "     </visual>"
              << "     <self_collide>0</self_collide>"
              << "     <kinematic>0</kinematic>"
              << "     <gravity>1</gravity>"
              << "    </link>"
              << "  </model>"
              << "</sdf>";
  index++;
  return newModelStr.str();
}

void makeMaze(std::vector<std::vector<double>> &maze_raw, std::vector<std::string> &sdf_models){
  std::string type;
  std::string color;
  std::vector <std::vector <double>> points;
  bool start_model = false;
  for (double i = 0; i < maze_raw.size(); i++){
    std::vector<double> line = maze_raw.at(i);
    for (double j = 0; j < line.size(); j++){
      if(line.at(j) && !start_model){
        start_model = true;
        points.push_back({i,j});
      }
      else if(!line.at(j) && start_model){
        start_model = false;
        points.push_back({i,j - 1});
      }

      if(start_model && (j == (line.size() - 1))){
        start_model = false;
        points.push_back({i,j});
      }
    }
  }
  type = "box";
  color = "White";
  for(int i = 0; i < points.size() - 1; i+=2){
    if((points.at(i)[0] != points.at(i+1)[0]) &&
       (points.at(i)[1] != points.at(i+1)[1])) continue;
    double size = sqrt((pow(points.at(i+1)[0] - points.at(i)[0], 2))+(pow(points.at(i+1)[1] - points.at(i)[1], 2)));
    double angle = atan2(static_cast<double>(points.at(i+1)[1] - points.at(i)[1]),
                         static_cast<double>(points.at(i+1)[0] - points.at(i)[0]));
    double center[2] = {(size/2.0)*cos(angle) +  points.at(i)[0], (size/2.0)*sin(angle) + points.at(i)[1]};
    if(!size){
      size = 2.0;
      angle = 0.0;
      center[0] = points.at(i)[0];
      center[1] = points.at(i)[1];
    }
    sdf_models.push_back(setGeometry(type, color, {center[0], center[1], 1.0}, {0.0, 0.0, angle}, {size, 0.2, 2.0}));
  }
}

void makePath(std::vector<std::vector<double>> &path_raw, std::vector<std::string> &sdf_models){
  std::string type;
  std::string color;
  std::vector <std::vector <double>> points;
  bool start_model = false;

  for (double i = 0; i < path_raw.size(); i++){
    std::vector<double> line = path_raw.at(i);
    for (double j = 0; j < line.size(); j++){
      if(line.at(j) && !start_model){
        start_model = true;
        points.push_back({i,j});
      }
      else if(!line.at(j) && start_model){
        start_model = false;
        points.push_back({i,j - 1});
      }
    }
  }

  type = "plane";
  color = "Yellow";
  for(int i = 0; i < points.size() - 1; i+=2){
    if((points.at(i)[0] != points.at(i+1)[0]) &&
       (points.at(i)[1] != points.at(i+1)[1])) continue;
    double size = sqrt((pow(points.at(i+1)[0] - points.at(i)[0], 2))+(pow(points.at(i+1)[1] - points.at(i)[1], 2)));
    double angle = atan2(static_cast<double>(points.at(i+1)[1] - points.at(i)[1]),
                         static_cast<double>(points.at(i+1)[0] - points.at(i)[0]));
    double center[2] = {(size/2.0)*cos(angle) +  points.at(i)[0], (size/2.0)*sin(angle) + points.at(i)[1]};
    if(!size){
      size = 2.0;
      angle = 0.0;
      center[0] = points.at(i)[0];
      center[1] = points.at(i)[1];
    }
    sdf_models.push_back(setGeometry(type, color, {center[0], center[1], 0.01}, {0.0, 0.0, angle}, {size, 0.2, 0.01}));
  }
}

void applyModel(std::vector<std::vector<double>> &model_pos, std::vector<std::string> &type, std::vector<std::string> &sdf_models){

  for (double i = 0; i < model_pos.size(); i++){
    std::vector<double> line = model_pos.at(i);
    for (double j = 0; j < line.size(); j++){
      if(line.at(j)){
        sdf_models.push_back(setModel(type.at(line.at(j) - 1), {i, j, 0.01}, {0.0, 0.0, 0.0}));
      }
    }
  }
}


void saveWorldFile(const std::string &filename, std::vector<std::string> &sdf_models)
{
  std::ofstream file(filename);
  if (!file.is_open())
  {
    std::cerr << "[ERROR] Could not open file for writing: " << filename << std::endl;
    return;
  }

  // Cabeçalho do mundo
  file << "<sdf version='1.6'>\n";
  file << "  <world name='default'>\n";

  file << "    "<< "<scene>\n";
  file << "    "<< " <ambient>0.4 0.4 0.4 1</ambient>\n";
  file << "    "<< " <background>0.7 0.7 0.7 1</background>\n";
  file << "    "<< " <shadows>true</shadows>\n";
  file << "    "<< "</scene>\n";

  file << "    "<< "<!-- A global light source -->\n";
  file << "    "<< "<include>\n";
  file << "    "<< "  <uri>model://sun</uri>\n";
  file << "    "<< "</include>\n";

  file << "    "<< "<!-- A ground plane -->\n";
  file << "    "<< "<include>\n";
  file << "    "<< "  <uri>model://ground_plane</uri>\n";
  file << "    "<< "</include>\n";

  file << "    "<< "<physics type='ode'>\n";
  file << "    "<< "  <gravity>0 0 -9.81</gravity>\n";
  file << "    "<< "  <real_time_update_rate>1000.0</real_time_update_rate>\n";
  file << "    "<< "  <max_step_size>0.001</max_step_size>\n";
  file << "    "<< "  <real_time_factor>1</real_time_factor>\n";
  file << "    "<< "  <ode>\n";
  file << "    "<< "    <solver>\n";
  file << "    "<< "      <type>quick</type>\n";
  file << "    "<< "      <iters>150</iters>\n";
  file << "    "<< "      <precon_iters>0</precon_iters>\n";
  file << "    "<< "      <sor>1.400000</sor>\n";
  file << "    "<< "      <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>\n";
  file << "    "<< "    </solver>\n";
  file << "    "<< "    <constraints>\n";
  file << "    "<< "      <cfm>0.00001</cfm>\n";
  file << "    "<< "      <erp>0.2</erp>\n";
  file << "    "<< "      <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>\n";
  file << "    "<< "      <contact_surface_layer>0.01000</contact_surface_layer>\n";
  file << "    "<< "    </constraints>\n";
  file << "    "<< "  </ode>\n";
  file << "    "<< "</physics>\n";

  // Inclui todos os modelos gerados dinamicamente
  for (const auto &model_sdf : sdf_models)
  {
    // Remove a tag <sdf> externa dos modelos
    size_t start = model_sdf.find("<model");
    size_t end = model_sdf.find("</model>") + 8;
    if (start != std::string::npos && end != std::string::npos)
    {
      file << "    " << model_sdf.substr(start, end - start) << "\n";
    }
  }

  // Fecha mundo e SDF
  file << "  </world>\n";
  file << "</sdf>\n";
  file.close();

  std::cout << "[INFO] Mundo salvo como: " << filename << std::endl;
}


int main(int argc, char **argv)
{

  std::vector<std::string> sdf_models;

  ros::init(argc, argv, "generate_line");
  ros::NodeHandle nh("~");

  std::cout << "[INFO] Get world filename" << std::endl;
  std::string filename;
  nh.getParam("/output", filename);

  std::cout << "[INFO] Get walls pos" << std::endl;
  auto maze_full = getMatrixParam(nh, "/maze");

  std::cout << "[INFO] Get line pos" << std::endl;
  auto path_full = getMatrixParam(nh, "/path");

  std::cout << "[INFO] Get model pos and type" << std::endl;
  auto type = getModelTypes(nh, "/models");
  auto model_pos = getMatrixParam(nh, "/model_pos");

  std::cout << "[INFO] Insert walls" << std::endl;
  makeMaze(maze_full, sdf_models);

  std::cout << "[INFO] Insert lines" << std::endl;
  makePath(path_full, sdf_models);

  std::cout << "[INFO] Insert models" << std::endl;
  applyModel(model_pos, type, sdf_models);

  saveWorldFile(filename, sdf_models);
  exit(0);
}