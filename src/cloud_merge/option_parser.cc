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

#include "option_parser.h"

#include <functional>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


using loadfunc = std::function<int(const std::string&, pcl::PointCloud<pcl::PointXYZ>&)>;
std::map<std::string, loadfunc> extension_map = {
	{".pcd", pcl::io::loadPCDFile<pcl::PointXYZ>},
	{".ply", pcl::io::loadPLYFile<pcl::PointXYZ>}
};

namespace OptionParser
{
// First input.
std::string input1 = "input1.pcd";

// Second input.
std::string input2 = "input2.pcd";

// list of cloud file input
std::vector<std::string> clouds_inputs;

// Output. The transformed second input.
std::string output = "";
// Default name for the '.ply' output file
std::string defaultPlyOutput = "output.ply";

// Transformation matrice.
std::string outputMat = "";
// Default name for the matrix output file
std::string defaultMatOutput = "transform.txt";

int align_iter_n = 40;

// Delta (see the paper).
double delta = 0.25;

// Estimated overlap (see the paper).
double overlap = 0.2;

// Maximum norm of RGB values between corresponded points. 1e9 means don't use.
double max_color = -1;

// Number of sampled points in both files. The 4PCS allows a very aggressive
// sampling.
int n_points = 800;

// Maximum angle (degrees) between corresponded normals.
double norm_diff = -1;

// Maximum allowed computation time.
int max_time_seconds = 60;

// number of neighbours for KDTree search
int knn_size = 30;

// number of neighbours to consider for region growing
int grow_nn_size = 30;

double norm_angle_diff = 8.0;

double curvature_thres = 5.0;

int min_cluster_size = 100;

bool skip_benchmarking = false;

static inline void printRegistrationParameters() {
	fprintf(stderr, "Registration Parameters:\n");
	fprintf(stderr, "\t[ -o overlap (%2.2f) ]\n", overlap);
	fprintf(stderr, "\t[ -d delta (%2.2f) ]\n", delta);
	fprintf(stderr, "\t[ -n n_points (%d) ]\n", n_points);
	fprintf(stderr, "\t[ -a norm_diff (%f) ]\n", norm_diff);
	// fprintf(stderr, "\t[ -c max_color_diff (%f) ]\n", max_color);
	fprintf(stderr, "\t[ -iter align_iter_n (%d) ]\n", align_iter_n);
	fprintf(stderr, "\t[ -t max_time_seconds (%d) ]\n", max_time_seconds);
	fprintf(stderr, "\t[ -m matrix_output (%s) ]\n", defaultMatOutput.c_str());
}

static inline void printPreprocessUsage() {
	fprintf(stderr, "Preprocessing Parameters:\n");
	fprintf(stderr, "\t[ -g grow_nn_size (%d) ]\n", grow_nn_size);
	fprintf(stderr, "\t[ -k knn_size (%d) ]\n", knn_size);
	fprintf(stderr, "\t[ -s norm_angle_diff (%2.2f) ]\n", norm_angle_diff);
	fprintf(stderr, "\t[ -c curvature_thres (%2.2f) ]\n", curvature_thres);
	fprintf(stderr, "\t[ -mc min_cluster_size (%d) ]\n", min_cluster_size);
}

void printUsage(char** argv) {
	fprintf(stderr, "\nUsage: %s -i input1 input2\n", argv[0]);
	fprintf(stderr, "OPTIONS:\n");

	printRegistrationParameters();
	printPreprocessUsage();
}

// will update the value of `curr` passed into this function
static void parse_file_name(int& curr, int argc, char** argv) {
	const std::string ext = ".pcd";

	// index starts from the value immediately followed by the flag
	for(int i = curr; i < argc; i++) {
		// reached the next flag
		if(argv[i][0] == '-') {
			break;
		}

		std::string fname = std::string(argv[i]);
		// Check if found
		std::string::size_type it = fname.rfind(ext);
		if(it != std::string::npos) {
			// Additional check: we want to be able to differentiate between .p and .png
			if((ext.size() - (fname.size() - it)) == 0) {
				clouds_inputs.push_back(fname);
			}
		}
	}

	// decrement 1 so that the index to be interpreted by `parse_argument()` will not be offsetted by 1
	curr--;
}

int parse_argument(int argc, char** argv) {
	for(int i = 1; i < argc; i++) {
		// super4pcs parameters
		if(!strcmp(argv[i], "-i")) {
			input1 = std::string(argv[++i]);
			input2 = std::string(argv[++i]);
		}
		else if(!strcmp(argv[i], "-f")) {
			parse_file_name(++i, argc, argv);
		}
		else if(!strcmp(argv[i], "-o")) {
			overlap = atof(argv[++i]);
			skip_benchmarking = true;
		}
		else if(!strcmp(argv[i], "-d")) {
			delta = atof(argv[++i]);
		}
		// else if(!strcmp(argv[i], "-c")) {
		// 	max_color = atof(argv[++i]);
		// }
		else if(!strcmp(argv[i], "-t")) {
			max_time_seconds = atoi(argv[++i]);
		}
		else if(!strcmp(argv[i], "-a")) {
			norm_diff = atof(argv[++i]);
		}
		else if(!strcmp(argv[i], "-n")) {
			n_points = atoi(argv[++i]);
		}
		else if(!strcmp(argv[i], "-iter")) {
			align_iter_n = atoi(argv[++i]);
		}
		else if(!strcmp(argv[i], "-r")) {
			output = argv[++i];
		}
		else if(!strcmp(argv[i], "-m")) {
			outputMat = argv[++i];
		}

		// parameter for cloud preprocessing
		else if(!strcmp(argv[i], "-g")) {
			grow_nn_size = atoi(argv[++i]);
		}
		else if(!strcmp(argv[i], "-k")) {
			knn_size = atoi(argv[++i]);
		}
		else if(!strcmp(argv[i], "-s")) {
			norm_angle_diff = atof(argv[++i]);
		}
		else if(!strcmp(argv[i], "-c")) {
			curvature_thres = atof(argv[++i]);
		}
		else if(!strcmp(argv[i], "-mc")) {
			min_cluster_size = atoi(argv[++i]);
		}

		else if(!strcmp(argv[i], "-h")) {
			return 1;
		}
		else if(argv[i][0] == '-') {
			std::cout << "Unknown flag: " << argv[i] << "\r\n";
			return -1;
		};
	}

	// if no output file (geometry/matrix) is set, force 3d mesh
	if(output.empty())
		output = defaultPlyOutput;

	if(outputMat.empty())
		outputMat = defaultMatOutput;

	return 0;
}

} // namespace OptionParser