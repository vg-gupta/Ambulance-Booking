#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <limits>
#include <algorithm>

struct Location {
    int x;
    int y;
};

struct Ambulance {
    Location location;
    double timeWithoutTraffic;
    double timeWithTraffic;
};

struct Hospital {
    Location location;
};

struct Node {
    Location location;
    double gScore;
    double hScore;
    double fScore;
    Node* cameFrom;

    Node(const Location& loc, double g, double h)
        : location(loc), gScore(g), hScore(h), fScore(g + h), cameFrom(nullptr) {}
};

std::vector<Ambulance> generateRandomAmbulanceLocations(int numAmbulances) {
    std::vector<Ambulance> ambulances;
    for (int i = 0; i < numAmbulances; ++i) {
        Ambulance ambulance;
        ambulance.location.x = rand() % 100;
        ambulance.location.y = rand() % 100;
        ambulance.timeWithoutTraffic = rand() % 100 + 1;
        ambulance.timeWithTraffic = rand() % 100 + 1;
        ambulances.push_back(ambulance);
    }
    return ambulances;
}

void bookAmbulance(const std::vector<Ambulance>& ambulances) {
    std::cout << "Enter your location (x y): ";
    Location userLocation;
    std::cin >> userLocation.x >> userLocation.y;

    std::cout << "Ambulances near your location:\n";
    for (size_t i = 0; i < ambulances.size(); ++i) {
        const Ambulance& ambulance = ambulances[i];
        std::cout << "Ambulance " << i + 1 << ": Location (" << ambulance.location.x << ", "
                  << ambulance.location.y << ")\n";
        std::cout << "  Estimated Time (Without Traffic): " << ambulance.timeWithoutTraffic << " minutes\n";
        std::cout << "  Estimated Time (With Traffic): " << ambulance.timeWithTraffic << " minutes\n";
    }

    int selection;
    std::cout << "Select an ambulance (1-" << ambulances.size() << "): ";
    std::cin >> selection;

    if (selection >= 1 && static_cast<size_t>(selection) <= ambulances.size()) {
        std::cout << "You selected Ambulance " << selection << ".\n";
        const Ambulance& selectedAmbulance = ambulances[selection - 1];
        std::cout << "Estimated time to reach you (Without Traffic): " << selectedAmbulance.timeWithoutTraffic << " minutes\n";
        std::cout << "Estimated time to reach you (With Traffic): " << selectedAmbulance.timeWithTraffic << " minutes\n";
        // Add the booking logic here
    } else {
        std::cout << "Invalid ambulance selection.\n";
    }
}

void bookHospital(std::vector<Hospital>& hospitals) {
    std::cout << "List of Hospitals:\n";
    for (size_t i = 0; i < hospitals.size(); ++i) {
        std::cout << i + 1 << ". Hospital " << i + 1 << "\n";
    }

    int selection;
    std::cout << "Select a hospital (1-" << hospitals.size() << "): ";
    std::cin >> selection;

    if (selection >= 1 && static_cast<size_t>(selection) <= hospitals.size()) {
        std::cout << "You selected Hospital " << selection << ". Booking in progress...\n";
        // Add the booking logic here
    } else {
        std::cout << "Invalid hospital selection.\n";
    }
}

std::vector<Location> findShortestPath(const Location& start, const Location& goal) {
    std::vector<Location> path;

    // A* algorithm implementation
    auto heuristic = [](const Location& a, const Location& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    };

    auto distance = [](const Location& a, const Location& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    };

    auto reconstructPath = [](Node* current) {
        std::vector<Location> path;
        while (current != nullptr) {
            path.push_back(current->location);
            current = current->cameFrom;
        }
        std::reverse(path.begin(), path.end());
        return path;
    };

    std::vector<Node*> openSet;
    std::vector<Node*> closedSet;

    Node* startNode = new Node(start, 0.0, heuristic(start, goal));
    openSet.push_back(startNode);

    while (!openSet.empty()) {
        // Find the node with the lowest fScore in the open set
        auto minIt = std::min_element(openSet.begin(), openSet.end(),
            [](const Node* a, const Node* b) { return a->fScore < b->fScore; });

        Node* current = *minIt;

        if (current->location.x == goal.x && current->location.y == goal.y) {
            path = reconstructPath(current);
            break;
        }

        openSet.erase(minIt);
        closedSet.push_back(current);

        // Generate neighbors of the current node
        std::vector<Location> neighbors = {
            {current->location.x - 1, current->location.y},
            {current->location.x + 1, current->location.y},
            {current->location.x, current->location.y - 1},
            {current->location.x, current->location.y + 1}
        };

        for (const Location& neighborLoc : neighbors) {
            if (neighborLoc.x < 0 || neighborLoc.x >= 100 || neighborLoc.y < 0 || neighborLoc.y >= 100) {
                continue;  // Skip out-of-bounds neighbors
            }

            Node* neighbor = new Node(neighborLoc, 0.0, heuristic(neighborLoc, goal));
            neighbor->gScore = current->gScore + distance(current->location, neighborLoc);

            // Check if the neighbor is in the closed set
            auto closedIt = std::find_if(closedSet.begin(), closedSet.end(),
                [neighbor](const Node* node) { return node->location.x == neighbor->location.x && node->location.y == neighbor->location.y; });

            if (closedIt != closedSet.end()) {
                continue;  // Skip if the neighbor is already evaluated
            }

            // Check if the neighbor is in the open set
            auto openIt = std::find_if(openSet.begin(), openSet.end(),
                [neighbor](const Node* node) { return node->location.x == neighbor->location.x && node->location.y == neighbor->location.y; });

            if (openIt == openSet.end()) {
                openSet.push_back(neighbor);
            } else if (neighbor->gScore >= (*openIt)->gScore) {
                continue;  // This path is not better
            }

            // This path is the best until now
            neighbor->cameFrom = current;
            neighbor->gScore = current->gScore + distance(current->location, neighbor->location);
            neighbor->fScore = neighbor->gScore + heuristic(neighbor->location, goal);
        }
    }

    // Cleanup: delete nodes in openSet and closedSet
    for (Node* node : openSet) {
        delete node;
    }

    for (Node* node : closedSet) {
        delete node;
    }

    return path;
}


int main() {
    srand(static_cast<unsigned>(time(nullptr)));

    std::vector<Ambulance> ambulances = generateRandomAmbulanceLocations(5);
    std::vector<Hospital> hospitals = { { {20, 30} }, { {50, 60} }, { {80, 10} } };

    int choice;
    do {
        std::cout << "Welcome to the Ambulance Booking System!\n";
        std::cout << "1. Book Ambulance\n";
        std::cout << "2. Book Hospital\n";
        std::cout << "3. Find Shortest Path to Hospital\n";
        std::cout << "4. Exit\n";
        std::cout << "Enter your choice (1-4): ";
        std::cin >> choice;

        switch (choice) {
            case 1:
                bookAmbulance(ambulances);
                break;
            case 2:
                bookHospital(hospitals);
                break;
            case 3: {
                std::cout << "Enter your location (x y): ";
                Location patientLocation;
                std::cin >> patientLocation.x >> patientLocation.y;

                std::cout << "Enter hospital location (x y): ";
                Location hospitalLocation;
                std::cin >> hospitalLocation.x >> hospitalLocation.y;

                std::vector<Location> path = findShortestPath(patientLocation, hospitalLocation);
                if (path.empty()) {
                    std::cout << "No path found!\n";
                } else {
                    std::cout << "Shortest path to the hospital:\n";
                    for (const Location& loc : path) {
                        std::cout << "(" << loc.x << ", " << loc.y << ") -> ";
                    
                    }
                    std::cout << "Hospital" << std::endl;
                }
                break;
            }
            case 4:
                std::cout << "Exiting the program.\n";
                break;
            default:
                std::cout << "Invalid choice. Please enter a number between 1 and 4.\n";
        }
    } while (choice != 4);

    return 0;
}
