#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <numeric>

using namespace std;

// Function to calculate the centroid of the simplex excluding the worst point
vector<double> calculateCentroid(const vector<vector<double>> &simplex, int worstIndex) {
    int dimensions = simplex[0].size();
    vector<double> centroid(dimensions, 0.0);

    for (size_t i = 0; i < simplex.size(); i++) {
        if (i == worstIndex) continue;
        for (int j = 0; j < dimensions; j++) {
            centroid[j] += simplex[i][j];
        }
    }

    for (int j = 0; j < dimensions; j++) {
        centroid[j] /= (simplex.size() - 1);
    }

    return centroid;
}

// Nelder-Mead optimization algorithm
vector<double> nelderMead(function<double(vector<double>)> objective, vector<vector<double>> initialSimplex, double tol = 1e-6, int maxIter = 1000) {
    int dimensions = initialSimplex[0].size();
    int n = dimensions + 1;
    vector<vector<double>> simplex = initialSimplex;
    vector<double> functionValues(n);

    // Calculate initial function values
    for (int i = 0; i < n; i++) {
        functionValues[i] = objective(simplex[i]);
    }

    for (int iter = 0; iter < maxIter; iter++) {
        // Sort simplex based on function values
        vector<int> indices(n);
        iota(indices.begin(), indices.end(), 0);
        sort(indices.begin(), indices.end(), [&](int a, int b) { return functionValues[a] < functionValues[b]; });

        vector<vector<double>> sortedSimplex(n);
        for (int i = 0; i < n; i++) {
            sortedSimplex[i] = simplex[indices[i]];
            functionValues[i] = objective(sortedSimplex[i]);
        }

        simplex = sortedSimplex;

        // Check for convergence
        double range = functionValues[n - 1] - functionValues[0];
        if (range < tol) break;

        // Compute centroid
        vector<double> centroid = calculateCentroid(simplex, n - 1);

        // Reflect the worst point
        vector<double> reflection(dimensions);
        for (int j = 0; j < dimensions; j++) {
            reflection[j] = centroid[j] + 1.0 * (centroid[j] - simplex[n - 1][j]);
        }
        double fReflection = objective(reflection);

        if (fReflection < functionValues[0]) {
            // Expansion
            vector<double> expansion(dimensions);
            for (int j = 0; j < dimensions; j++) {
                expansion[j] = centroid[j] + 2.0 * (reflection[j] - centroid[j]);
            }
            double fExpansion = objective(expansion);

            if (fExpansion < fReflection) {
                simplex[n - 1] = expansion;
            } else {
                simplex[n - 1] = reflection;
            }
        } else if (fReflection < functionValues[n - 2]) {
            // Accept reflection
            simplex[n - 1] = reflection;
        } else {
            // Contraction
            vector<double> contraction(dimensions);
            for (int j = 0; j < dimensions; j++) {
                contraction[j] = centroid[j] + 0.5 * (simplex[n - 1][j] - centroid[j]);
            }
            double fContraction = objective(contraction);

            if (fContraction < functionValues[n - 1]) {
                simplex[n - 1] = contraction;
            } else {
                // Shrink the simplex
                for (int i = 1; i < n; i++) {
                    for (int j = 0; j < dimensions; j++) {
                        simplex[i][j] = simplex[0][j] + 0.5 * (simplex[i][j] - simplex[0][j]);
                    }
                }
            }
        }

        // Update function values
        for (int i = 0; i < n; i++) {
            functionValues[i] = objective(simplex[i]);
        }
    }

    // Return the best point
    return simplex[0];
}

int main() {
    // Example usage: minimize Rosenbrock function
    auto rosenbrock = [](vector<double> x) {
        return pow(1 - x[0], 2) + 100 * pow(x[1] - x[0] * x[0], 2);
    };

    // Initial simplex
    vector<vector<double>> initialSimplex = {
        {0.0, 0.0},
        {1.0, 0.0},
        {0.0, 1.0}
    };

    // Run Nelder-Mead algorithm
    vector<double> result = nelderMead(rosenbrock, initialSimplex);

    // Print result
    cout << "Minimum found at: ";
    for (double val : result) {
        cout << val << " ";
    }
    cout << endl;
    sprintf(cout);

    return 0;
}