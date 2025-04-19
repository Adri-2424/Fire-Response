#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <chrono>
#include <utility>
#include <map>
#include <iomanip>
#include "quadTree.cpp"

//struct to intialize stuff for parsing for the columns in the csv
struct FireIncident {
    std::string id;
    double latitude;
    double longitude;
    double baseLat;
    double baseLon;
    double distance;

    FireIncident(const std::string& id_, double lat, double lon, double bLat, double bLon, double dist)
        : id(id_), latitude(lat), longitude(lon), baseLat(bLat), baseLon(bLon), distance(dist) {}
};

std::vector<FireIncident> parseCSV(const std::string& filename) {
    std::vector<FireIncident> incidents;
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        std::string id;
        double lat, lon, bLat, bLon, dist;

        std::getline(ss, token, ',');
        try {
            dist = std::stod(token);
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid number in DistanceFromBase: '" << token << "'\n";
            continue;
        } catch (const std::out_of_range& e) {
            std::cerr << "Number out of range in DistanceFromBase: '" << token << "'\n";
            continue;
        }

        //parsing columns currently skipping date and time cause we dont need it rn
        std::getline(ss, id, ',');
        for (int i = 0; i < 6; ++i) std::getline(ss, token, ','); // Skip Hour → Unit
        std::getline(ss, token, ','); lon = std::stod(token);     // Longitude
        std::getline(ss, token, ','); lat = std::stod(token);     // Latitude
        for (int i = 0; i < 2; ++i) std::getline(ss, token, ','); // Skip Date, Time
        std::getline(ss, token, ','); bLat = std::stod(token);    // BaseLatitude
        std::getline(ss, token, ','); bLon = std::stod(token);    // BaseLongitude
        std::getline(ss, token, ','); dist = std::stod(token);    // DistanceFromBase

        incidents.emplace_back(id, lat, lon, bLat, bLon, dist);
    }
    //double check if loaded later lmao
    return incidents;
}

//struct code modified from https://www.baeldung.com/cs/k-d-trees wbesite
struct KDNode {
    FireIncident data;
    KDNode* left;
    KDNode* right;

    KDNode(FireIncident d) : data(d), left(nullptr), right(nullptr) {}
};

class KDTree {
public:
    KDNode* root = nullptr;

    KDNode* insert(KDNode* node, FireIncident point, int depth = 0) {
        if (!node) return new KDNode(point);

        //axis checks where we are at (x or y) and then moves the daughter nodes accordingly
        int axis = depth % 2;
        if ((axis == 0 && point.latitude < node->data.latitude) ||
            (axis == 1 && point.longitude < node->data.longitude)) {
            node->left = insert(node->left, point, depth + 1);
            } else {
                node->right = insert(node->right, point, depth + 1);
            }

        return node;
    }

    //build the tree
    void build(const std::vector<FireIncident>& points) {
        for (const auto& pt : points)
            root = insert(root, pt);
    }

    //nearest neighbor algorithm for calculating closest points
    FireIncident nearest(KDNode* node, double lat, double lon, int depth = 0,
                       FireIncident best = FireIncident("", 0, 0, 0, 0, std::numeric_limits<double>::max())) {

        if (!node) return best;

        double dist = std::hypot(lat - node->data.latitude, lon - node->data.longitude);
        if (dist < best.distance)
            best = FireIncident(node->data.id, node->data.latitude, node->data.longitude,
                                node->data.baseLat, node->data.baseLon, dist);

        //axis stuff again because then we know which side of the tree were going down and by which way x or y
        int axis = depth % 2;
        KDNode *next = nullptr, *other = nullptr;

        if ((axis == 0 && lat < node->data.latitude) ||
            (axis == 1 && lon < node->data.longitude)) {
            next = node->left;
            other = node->right;
            } else {
                next = node->right;
                other = node->left;
            }

        //save the current best and make sure if the next best is better then replace it for best
        best = nearest(next, lat, lon, depth + 1, best);
        double axis_diff = (axis == 0 ? lat - node->data.latitude : lon - node->data.longitude);
        if (std::abs(axis_diff) < best.distance)
            best = nearest(other, lat, lon, depth + 1, best);

        return best;
    }

    //find nearest using the distance formula and calculate its nearest point
    void findNearestUnit(const FireIncident& nearestIncident, const std::map<std::string, std::pair<double, double>>& unit_coordinates) {
        std::string closestUnit;
        double minDist = std::numeric_limits<double>::max();
        for (const auto& unit : unit_coordinates) {
            double dist = std::hypot(nearestIncident.latitude - unit.second.first, nearestIncident.longitude - unit.second.second);
            if (dist < minDist) {
                minDist = dist;
                closestUnit = unit.first;
            }
        }

        std::cout << "The nearest unit likely to respond is: " << closestUnit << "\n";
    }
};

Point toQuadPoint(double latitude, double longitude){
    return Point{static_cast<int>(latitude*10000), static_cast<int>(longitude*10000)};
}

//ignore for now cause I didnt do a quadtree yet
void benchmark(const std::vector<FireIncident>& data) {
    KDTree kdtree;
    kdtree.build(data);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = kdtree.nearest(kdtree.root, 29.7, -82.3);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "KDTree nearest took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms\n";

    //QuadTree
    Boundary boundary = {0, 0, 1000000, 1000000};
    QuadTree qt(boundary);

    for (auto incident : data){
        qt.insert(toQuadPoint(incident.latitude, incident.longitude));
    }

    Point target = toQuadPoint(29.7, -82.3);
    double bestDist = numeric_limits<double>::max();

    auto startQT  = chrono::high_resolution_clock::now();

    Point close = qt.closest(target, bestDist, {0,0});

    auto endQT  = chrono::high_resolution_clock::now();

    cout << "QuadTree nearest took " << chrono::duration_cast<chrono::microseconds>(endQT - startQT).count() << " ms\n";

    //if wanted to print the closest point
    // cout << "QuadTree closest point: (" << close.x << ", " << close.y << ")\n";
}
// for finding the coordinates to the areas with the highest incident rate
void getHigherIncidents(vector<FireIncident> incidents, int incidentThreshold){
    if (incidentThreshold <= 0){
        cout << "Invalid threshold, must be greater than 0" << endl;
        return;
    }

    // if there is an incident, add the location coordinates & iterate count of incidents
    map<pair<int, int>, int> regionCountMap;
    for (auto incident : incidents){
        int lat = static_cast<int>(incident.latitude * 100);
        int lon = static_cast<int>(incident.longitude * 100);
        regionCountMap[{lat, lon}]++;
    }

    // print out high incident areas
    cout << "High Incident Areas: " << endl;
    for (auto region : regionCountMap){
        int EMSIncidents = region.second;
        if (EMSIncidents >= incidentThreshold){
            int EMSResponse = static_cast<int>(ceil(static_cast<double>(EMSIncidents) / incidentThreshold));
            cout << "(" << region.first.first / 100.0 << ", ";
            cout << region.first.second / 100.0 << ")" << endl;
            cout << EMSResponse << " more EMS Units on standby" << endl;
        }
    }
}

int main() {
    std::string filename = "cleaned_incident_data.csv";
    auto incidents = parseCSV(filename);

    std::cout << "Loaded " << incidents.size() << " incidents" << std::endl;

    KDTree kdtree;
    kdtree.build(incidents);

    double userLat, userLon;
    std::cout << "Enter Latitude of the fire incident: ";
    std::cin >> userLat;
    std::cout << "Enter Longitude of the fire incident: ";
    std::cin >> userLon;

    FireIncident nearestIncident = kdtree.nearest(kdtree.root, userLat, userLon);
    std::cout << "The nearest fire incident to the location (" << userLat << ", " << userLon << ") is:\n"
              << "Incident ID: " << nearestIncident.id << "\n"
              << "Location: (" << nearestIncident.latitude << ", " << nearestIncident.longitude << ")\n"
              << "Distance from base: " << nearestIncident.distance << " km\n";

    std::map<std::string, std::pair<double, double>> unit_coordinates;

    //syntehsized locations of fire units
    unit_coordinates["E7"] = {29.700735771993592, -82.38651442738922};
    unit_coordinates["SQ3"] = {29.665023513467546, -82.29958933643647};
    unit_coordinates["E3"] = {29.66395757157737, -82.30181092608116};
    unit_coordinates["E2"] = {29.629974283595164, -82.35531600790573};
    unit_coordinates["Q9"] = {29.62361312432365, -82.3706184157027};
    unit_coordinates["Q8"] = {29.69829338320167, -82.37038952707218};
    unit_coordinates["TW1"] = {29.6558004642621, -82.32716385331416};
    unit_coordinates["E5"] = {29.68089304705855, -82.3350131946103};
    unit_coordinates["E1"] = {29.6535648986405, -82.32892497332307};
    unit_coordinates["E4"] = {29.65677821533309, -82.38085479231843};
    unit_coordinates["SQ1"] = {29.65274296122673, -82.3274143683069};
    unit_coordinates["SQ2"] = {29.634307529137704, -82.37794834140252};
    unit_coordinates["HZ2"] = {29.667737620635176, -82.34663823759034};
    unit_coordinates["Q2"] = {29.634968679459902, -82.35666260626792};
    unit_coordinates["TW2"] = {29.633897140286848, -82.35655893525745};
    unit_coordinates["CR6-1"] = {29.68157895, -82.276542475};
    unit_coordinates["L9"] = {29.628166453149003, -82.38419263932411};
    unit_coordinates["Q1"] = {29.65610122258656, -82.32648061217381};
    unit_coordinates["TR8"] = {29.69629839603961, -82.36386959405941};
    unit_coordinates["DC1"] = {29.65152004373069, -82.33963253032567};
    unit_coordinates["CRP1"] = {29.664385421875, -82.33400179340278};
    unit_coordinates["DC2"] = {29.690520165011428, -82.39215404813429};
    unit_coordinates["TR9"] = {29.6273168984375, -82.3834842890625};
    unit_coordinates["CRP2"] = {29.66423636, -82.33255536};
    unit_coordinates["TR2"] = {29.62992382669149, -82.3576894853617};
    unit_coordinates["MRU4"] = {29.652504392, -82.34277542};
    unit_coordinates["CR6-3"] = {29.678154608695653, -82.28699813043478};
    unit_coordinates["DC3"] = {29.6548518, -82.35892079999999};
    unit_coordinates["SQ9"] = {29.623905466154763, -82.37939707866005};
    unit_coordinates["TR1"] = {29.656132808510645, -82.32722745212766};
    unit_coordinates["E9"] = {29.654222857142862, -82.34809002380952};
    unit_coordinates["SQ4"] = {29.655778666666667, -82.32370466666667};
    unit_coordinates["CR61"] = {29.679144866666668, -82.27718196666666};
    unit_coordinates["HZ1"] = {29.67977952801816, -82.33657692804908};

    kdtree.findNearestUnit(nearestIncident, unit_coordinates);

    std::cout << "Unit coordinates have been populated." << std::endl;

    benchmark(incidents);

    int incidentThreshold = 0;
    cout << "Please input a threshold for incidents: " << endl;
    cin >> incidentThreshold;

    getHigherIncidents(incidents, incidentThreshold);

    return 0;
}
