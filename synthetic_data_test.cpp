#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

struct Observation {
    std::vector<double> features;
    bool label;
};

double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        sum += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return std::sqrt(sum);
}

std::vector<Observation> updateTrainingSet(const std::vector<Observation>& xl, const std::vector<Observation>& xtest,
                                           const std::vector<std::vector<double>>& clust_cents_occ,
                                           const std::vector<std::vector<double>>& clust_cents_unocc,
                                           double thresh_occ, double thresh_unocc) {
    std::vector<Observation> updatedTrainingSet = xl;

    for (const auto& obs : xtest) {
        double min_dist_occ = std::numeric_limits<double>::max();
        double min_dist_unocc = std::numeric_limits<double>::max();

        for (const auto& center : clust_cents_occ) {
            double dist = euclideanDistance(obs.features, center);
            if (dist < min_dist_occ) {
                min_dist_occ = dist;
            }
        }

        for (const auto& center : clust_cents_unocc) {
            double dist = euclideanDistance(obs.features, center);
            if (dist < min_dist_unocc) {
                min_dist_unocc = dist;
            }
        }

        if (min_dist_occ < thresh_occ) {
            Observation newObs = {obs.features, true};
            if (updatedTrainingSet.size() >= 1000) {
                auto farthestObs = std::max_element(updatedTrainingSet.begin(), updatedTrainingSet.end(),
                                                    [&](const Observation& a, const Observation& b) {
                                                        return euclideanDistance(a.features, obs.features) <
                                                               euclideanDistance(b.features, obs.features);
                                                    });
                *farthestObs = newObs;
            } else {
                updatedTrainingSet.push_back(newObs);
            }
        } else if (min_dist_unocc < thresh_unocc) {
            Observation newObs = {obs.features, false};
            if (updatedTrainingSet.size() >= 1000) {
                auto farthestObs = std::max_element(updatedTrainingSet.begin(), updatedTrainingSet.end(),
                                                    [&](const Observation& a, const Observation& b) {
                                                        return euclideanDistance(a.features, obs.features) <
                                                               euclideanDistance(b.features, obs.features);
                                                    });
                *farthestObs = newObs;
            } else {
                updatedTrainingSet.push_back(newObs);
            }
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

    // Use the updated training set for KNN classification
    // ...

    return 0;
}
