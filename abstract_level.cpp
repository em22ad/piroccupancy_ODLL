#include <vector>
#include <algorithm>
#include <cmath>
#include <random>

using namespace std;

// K-means clustering function
vector<vector<double>> kmeans(vector<vector<double>>& data, int k) {

  // Initialize cluster centers randomly  
  default_random_engine gen;
  uniform_real_distribution<double> dis(0,1);

  vector<vector<double>> centers(k);
  for(auto& center: centers) {
    for(int i=0; i<data[0].size(); i++) {
      center.push_back(dis(gen));
    }
  }

  // Iterate until convergence
  while(true) {

    // Assign points to closest center
    vector<int> assignments(data.size()); 
    for(int i=0; i<data.size(); i++) {
      double minDist = INT_MAX;
      int closest = 0;
      for(int j=0; j<centers.size(); j++) {
        double dist = euclideanDist(data[i], centers[j]);
        if(dist < minDist) {
          minDist = dist;
          closest = j;
        }  
      }
      assignments[i] = closest;
    }

    // Recalculate centers
    for(int i=0; i<centers.size(); i++) {
      centers[i].clear();
    }

    for(int i=0; i<assignments.size(); i++) {
      for(int j=0; j<data[i].size(); j++) {
        centers[assignments[i]][j] += data[i][j];  
      }
    }

    for(int i=0; i<centers.size(); i++) {
      for(int j=0; j<centers[i].size(); j++) {
        centers[i][j] /= data.size();
      }
    }

    // Check convergence
    if(centers converged) break; 
  }

  return centers;
}

// Rest of the algorithm 1 function
void labelGenerator(vector<vector<double>>& xtest, ...) {

  vector<vector<double>> clusterCenters = kmeans(xl, k);

  // Remaining steps
}
