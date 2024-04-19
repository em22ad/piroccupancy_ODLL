#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

struct Observation {
    std::vector<double> features;
    bool label;
};

double euclideanDistance(const std::vector<double>& v1, const std::vector<double>& v2) {
    double sum = 0.0;
    for (size_t i = 0; i < v1.size(); ++i) {
        sum += std::pow(v1[i] - v2[i], 2);
    }
    return std::sqrt(sum);
}

double calculateWSS(const std::vector<Observation>& dataset, int k) {
    double wss = 0.0;
    for (const auto& obs : dataset) {
        std::vector<std::pair<double, bool>> distances;
        for (const auto& trainObs : dataset) {
            if (&obs != &trainObs) {
                double dist = euclideanDistance(obs.features, trainObs.features);
                distances.emplace_back(dist, trainObs.label);
            }
        }
        std::sort(distances.begin(), distances.end());
        for (int i = 0; i < k && i < distances.size(); ++i) {
            wss += std::pow(distances[i].first, 2);
        }
    }
    return wss;
}

int elbowSearch(const std::vector<Observation>& dataset, int maxK) {
    std::vector<double> wssValues;
    for (int k = 1; k <= maxK; ++k) {
        double wss = calculateWSS(dataset, k);
        wssValues.push_back(wss);
    }
    int optimalK = 1;
    double maxDiff = 0.0;
    for (int i = 1; i < wssValues.size() - 1; ++i) {
        double diff = std::abs(wssValues[i - 1] - 2 * wssValues[i] + wssValues[i + 1]);
        if (diff > maxDiff) {
            maxDiff = diff;
            optimalK = i + 1;
        }
    }
    return optimalK;
}

std::vector<Observation> updateTrainingSet(const std::vector<Observation>& xl, const std::vector<Observation>& xtest,
                                           const std::vector<std::vector<double>>& clust_cents_occ,
                                           const std::vector<std::vector<double>>& clust_cents_unocc,
                                           double thresh_occ, double thresh_unocc) {
    std::vector<Observation> updatedTrainingSet = xl;

    for (const auto& testObs : xtest) {
        bool isOccupied = false;
        for (const auto& centOcc : clust_cents_occ) {
            double dist = euclideanDistance(testObs.features, centOcc);
            if (dist < thresh_occ) {
                isOccupied = true;
                break;
            }
        }

        bool isUnoccupied = false;
        for (const auto& centUnocc : clust_cents_unocc) {
            double dist = euclideanDistance(testObs.features, centUnocc);
            if (dist < thresh_unocc) {
                isUnoccupied = true;
                break;
            }
        }

        if (isOccupied && !isUnoccupied) {
            Observation newObs = testObs;
            newObs.label = true;
            updatedTrainingSet.push_back(newObs);
        } else if (!isOccupied && isUnoccupied) {
            Observation newObs = testObs;
            newObs.label = false;
            updatedTrainingSet.push_back(newObs);
        }
    }

    return updatedTrainingSet;
}

int main() {
    // Initial training dataset
    std::vector<Observation> xl = {
        {{1.0, 2.0, 3.0}, true},
        {{4.0, 5.0, 6.0}, false},
        // ...
    };

    // Unlabeled observations
    std::vector<Observation> xtest = {
        {{7.0, 8.0, 9.0}, false},
        {{10.0, 11.0, 12.0}, false},
        // ...
    };

    // Cluster centers for occupied and unoccupied classes
    std::vector<std::vector<double>> clust_cents_occ = {
        {1.5, 2.5, 3.5},
        // ...
    };

    std::vector<std::vector<double>> clust_cents_unocc = {
        {4.5, 5.5, 6.5},
        // ...
    };

    double thresh_occ = 1.0;
    double thresh_unocc = 1.0;

    std::vector<Observation> updatedTrainingSet = updateTrainingSet(xl, xtest, clust_cents_occ, clust_cents_unocc, thresh_occ, thresh_unocc);

    // Use the elbow search method to find the optimal value of k
    int maxK = 10;
    int optimalK = elbowSearch(updatedTrainingSet, maxK);
    std::cout << "Optimal value of k: " << optimalK << std::endl;

    // Perform KNN classification using the optimal value of k
    for (const auto& testObs : xtest) {
        std::vector<std::pair<double, bool>> distances;
        for (const auto& trainObs : updatedTrainingSet) {
            double dist = euclideanDistance(testObs.features, trainObs.features);
            distances.emplace_back(dist, trainObs.label);
        }
        std::sort(distances.begin(), distances.end());
        int occupiedCount = 0;
        int unoccupiedCount = 0;
        for (int i = 0; i < optimalK && i < distances.size(); ++i) {
            if (distances[i].second) {
                occupiedCount++;
            } else {
                unoccupiedCount++;
            }
        }
        bool predictedLabel = (occupiedCount > unoccupiedCount);
        std::cout << "Test Observation: ";
        for (const auto& feature : testObs.features) {
            std::cout << feature << " ";
        }
        std::cout << "Predicted Label: " << (predictedLabel ? "Occupied" : "Unoccupied") << std::endl;
    }

    return 0;
}
