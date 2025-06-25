#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Pose3.hh>
#include <array>

void setSDF(const std::string &type, const std::string &color, ignition::math::Vector3d pose, ignition::math::Vector3d ori, ignition::math::Vector3d size){
  static long int index = 0;
  // Cria nó e inicializa
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Cria publisher para ~/factory
  gazebo::transport::PublisherPtr factoryPub =
      node->Advertise<gazebo::msgs::Factory>("~/factory");

  factoryPub->WaitForConnection();

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

  // Monta e envia a mensagem para a factory
  gazebo::msgs::Factory msg;
  msg.set_sdf(newModelStr.str());
  factoryPub->Publish(msg);
  std::cout << "[INFO]  Model enviado para o simulador." << std::endl;
  index++;
}

void makeMaze(std::vector<std::vector<double>> &maze_raw){
  std::string type;
  std::string color;
  std::vector <std::vector <double>> points;
  bool start_model = false;

  for (int i = 0; i < maze_raw.size(); i++){
    std::vector<double> line = maze_raw.at(i);
    for (int j = 0; j < line.size(); j++){
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
    setSDF(type, color, {center[0], center[1], 1.0}, {0.0, 0.0, angle}, {size, 0.2, 2.0});
    std::cout << "Size:  " <<size<< "|   Angle:  " <<angle*(180/M_PI)<< std::endl;
  }
}

void makePath(std::vector<std::vector<double>> &path_raw){
  std::string type;
  std::string color;
  std::vector <std::vector <double>> points;
  bool start_model = false;

  for (int i = 0; i < path_raw.size(); i++){
    std::vector<double> line = path_raw.at(i);
    for (int j = 0; j < line.size(); j++){
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
    setSDF(type, color, {center[0], center[1], 0.01}, {0.0, 0.0, angle}, {size, 0.2, 0.01});
    std::cout << "Size:  " <<size<< "|   Angle:  " <<angle*(180/M_PI)<< std::endl;
  }
}

int main(int argc, char **argv)
{
  // Inicializa o cliente Gazebo
  gazebo::client::setup();
  std::vector<std::vector<double>> maze_full = {{ {1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0},
                                                  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                  {1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
                                                  {1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
                                                  {1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
                                                  {1,0,0,1,0,0,1,1,1,1,1,1,1,1,1,0,0,1,0,0},
                                                  {1,0,0,1,0,0,1,0,0,0,0,0,0,0,1,0,0,1,0,0},
                                                  {1,0,0,1,0,0,1,0,0,0,0,0,0,0,1,0,0,1,0,0},
                                                  {1,0,0,1,0,0,1,0,0,1,1,1,0,0,1,0,0,1,0,0},
                                                  {1,0,0,1,0,0,1,0,0,0,0,1,0,0,1,0,0,1,0,0},
                                                  {1,0,0,1,0,0,1,0,0,0,0,1,0,0,1,0,0,1,0,0},
                                                  {1,0,0,1,0,0,1,0,0,0,0,1,0,0,1,0,0,1,0,0},
                                                  {1,0,0,1,0,0,1,1,1,1,1,1,0,0,1,0,0,1,0,0},
                                                  {1,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0},
                                                  {1,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0},
                                                  {1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0},
                                                  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
                                                  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
                                                  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0} }};

  makeMaze(maze_full);

  std::vector<std::vector<double>> path_full = {{ {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
                                                  {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                  {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                  {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                  {0,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
                                                  {0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
                                                  {0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
                                                  {0,1,0,0,0,1,0,0,1,1,1,1,1,1,0,0,1,0,0,0},
                                                  {0,1,0,0,0,1,0,0,1,0,0,0,0,1,0,0,1,0,0,0},
                                                  {0,1,0,0,0,1,0,0,1,0,0,0,0,1,0,0,1,0,0,0},
                                                  {0,1,0,0,0,1,0,0,1,0,1,0,0,1,0,0,1,0,0,0},
                                                  {0,1,0,0,0,1,0,0,1,1,1,0,0,1,0,0,1,0,0,0},
                                                  {0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,1,0,0,0},
                                                  {0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,1,0,0,0},
                                                  {0,1,0,0,0,1,1,1,1,1,1,1,1,1,0,0,1,0,0,0},
                                                  {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
                                                  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
                                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} }};

  makePath(path_full);

  //type = "plane";
  //color = "Yellow";
  //setSDF(type, color, {1.0, 4.0, 0.01}, {0.2, 10.0, 0.0});
  gazebo::client::shutdown();
  return 0;
}