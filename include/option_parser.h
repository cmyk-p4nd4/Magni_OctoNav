// Copyright 2017 Nicolas Mellado
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// -------------------------------------------------------------------------- //
//
// Authors: Nicolas Mellado
//
// This file is part of the implementation of the 4-points Congruent Sets (4PCS)
// algorithm presented in:
//
// Super 4PCS: Fast Global Pointcloud Registration via Smart Indexing
// Nicolas Mellado, Dror Aiger, Niloy J. Mitra
// Symposium on Geometry Processing 2014.
//
// Given two sets of points in 3-space, P and Q, the algorithm applies RANSAC
// in roughly O(n^2) time instead of O(n^3) for standard RANSAC, using an
// efficient method based on invariants, to find the set of all 4-points in Q
// that can be matched by rigid transformation to a given set of 4-points in P
// called a base. This avoids the need to examine all sets of 3-points in Q
// against any base of 3-points in P as in standard RANSAC.
// The algorithm can use colors and normals to speed-up the matching
// and to improve the quality. It can be easily extended to affine/similarity
// transformation but then the speed-up is smaller because of the large number
// of congruent sets. The algorithm can also limit the range of transformations
// when the application knows something on the initial pose but this is not
// necessary in general (though can speed the runtime significantly).

// Home page of the 4PCS project (containing the paper, presentations and a
// demo): http://graphics.stanford.edu/~niloy/research/fpcs/fpcs_sig_08.html
// Use google search on "4-points congruent sets" to see many related papers
// and applications.

#pragma once

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string.h>
#include <string>
#include <vector>

namespace OptionParser
{
// First input.
extern std::string input1;

// Second input.
extern std::string input2;

extern std::vector<std::string> clouds_inputs;

// Output. The transformed second input.
extern std::string output;
// Default name for the '.ply' output file
extern std::string defaultPlyOutput;

// Transformation matrice.
extern std::string outputMat;
// Default name for the matrix output file
extern std::string defaultMatOutput;

extern int align_iter_n;

// Delta (see the paper).
extern double delta;

// Estimated overlap (see the paper).
extern double overlap;

// Maximum norm of RGB values between corresponded points. 1e9 means don't use.
extern double max_color;

// Number of sampled points in both files. The 4PCS allows a very aggressive
// sampling.
extern int n_points;

// Maximum angle (degrees) between corresponded normals.
extern double norm_diff;

// Maximum allowed computation time.
extern int max_time_seconds;

// number of neighbours for KDTree search
extern int knn_size;

// number of neighbours to consider for region growing
extern int grow_nn_size;

// maximum angle (degrees) between normals for region growing
extern double norm_angle_diff;

extern double curvature_thres;

extern int min_cluster_size;

extern bool skip_benchmarking;

void printUsage(char** argv);

int parse_argument(int argc, char** argv);

template <typename OptionType>
inline bool setOptionsFromArgs(OptionType& options) {
	bool overlapOk = options.configureOverlap(overlap);
	if(!overlapOk) {
		std::cout << "Invalid Overlapp configuration!\r\n";
		return false;
	}
	options.sample_size = n_points;
	options.max_normal_difference = norm_diff;
	options.max_color_distance = max_color;
	options.max_time_seconds = max_time_seconds;
	options.delta = delta;

	return true;
}

} // namespace OptionParser