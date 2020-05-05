
#include "pure_pursuit.hpp"

using namespace std;

vector<float> PurePursuit::getWaypointDistances(const geometry_msgs::Pose &pose)
{
    geometry_msgs::TransformStamped world_to_car_msg_stamped = Transforms::WorldToCarTransform(pose);
    float carAngle = Transforms::getCarOrientation(pose);

    // need to think about this
    float upperAngle = fmod(carAngle+M_PI/2,M_PI);
    float lowerAngle = fmod(carAngle-M_PI/2,M_PI);
    if (lowerAngle > upperAngle) {
        swap(lowerAngle, upperAngle);
    }
    vector<float> distances;
    for (auto itr = waypoints_.begin(); itr != waypoints_.end(); itr++) {
        pair<float, float> point = itr->getPair();
        pair<float, float> transformedPoint = Transforms::TransformPoint(point, world_to_car_msg_stamped);
        float x = transformedPoint.first;
        float y = transformedPoint.second;
        if (x>0) {
            distances.push_back(sqrt(x*x+y*y));
        } else {
            distances.push_back(numeric_limits<float>::max());
        }
    }
    return distances;
}
PurePursuit::PurePursuit(float lookahead) : lookahead_(lookahead)
{}
PurePursuit::~PurePursuit()
{}

bool PurePursuit::readCMA_ES(string filename)
{
    string path = ros::package::getPath("milestone-3")+"/"+filename;
    // string path = "/home/saumya/team3_ws/src/F110-Final/fooxx.csv";
    cout << path << endl;
    ifstream input(path);
    string coordX, coordY;
    vector<pair<float,float>> temp;
    if (input.is_open()) {
        while(getline(input,coordX,',')) {
            getline(input,coordY);
            temp.push_back(pair<float,float>(stof(coordX),stof(coordY)));
        }
    } else {
        cout << "Please run this from the root catkin_ws directory" << endl;
        return false;
    }
    for (int i = 0; i < temp.size(); i++) {
        float prev_x = temp[(i-1)%temp.size()].first;
        float prev_y = temp[(i-1)%temp.size()].second;
        float x = temp[i].first;
        float y = temp[i].second;
        float ori = atan2(y-prev_y,x-prev_x);
        State state;
        state.SetX(x);
        state.SetY(y);
        state.SetOri(ori);
        waypoints_.push_back(state);
    }
    return true;
}

int PurePursuit::getClosestIdx(vector<float> &distances, float lookahead)
{
    float minDistance = numeric_limits<float>::max();
    float argminDist = -1;
    for (int i = 0; i < distances.size(); i++) {
        float currentDistance = distances[i] - lookahead_;
        float ori = waypoints_[i].ori();
        if (currentDistance >= 0 && currentDistance < minDistance) {// && ori >= lowerAngle && ori <= upperAngle) {
            argminDist = i;
            minDistance = currentDistance;
        }
    }
    return argminDist;
}

bool PurePursuit::isPathCollisionFree(const geometry_msgs::Pose &pose, OccGrid &occ_grid)
{
    vector<float> distances = getWaypointDistances(pose);
    int startingIdx = getClosestIdx(distances, 0);
    int endingIdx = getClosestIdx(distances, lookahead_);

    for (int i = startingIdx; i < endingIdx; i++) {
        if (!occ_grid.CheckCollision(waypoints_[i].getPair(),waypoints_[i+1].getPair())) {
            return false;
        }
    }
    return true;
}
