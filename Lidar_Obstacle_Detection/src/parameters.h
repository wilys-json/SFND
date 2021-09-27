#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <utility>

using std::string;
using std::ifstream;
using std::vector;
using std::istringstream;
using std::copy;

// Keywords
const string PCD_FILE{"PCD_FILE"};
const string PCD_FOLDER{"PCD_FOLDER"};
const string FILTER_RES{"FILTER_RES"};
const string MIN_POINT{"MIN_POINT"};
const string MAX_POINT{"MAX_POINT"};
const string MAX_ITER{"MAX_ITER"};
const string DISTANCE_THRESHOLD{"DISTANCE_THRESHOLD"};
const string CLUSTER_TOL{"CLUSTER_TOL"};
const string CLUSTER_MIN{"CLUSTER_MIN"};
const string CLUSTER_MAX{"CLUSTER_MAX"};

namespace Parameters {


  const string paramDoc{"../src/params"};

  // Template function to
  template <typename T>
  vector<T> getValues(const string& targetKey) {
    string line, key;
    T value;
    vector<T> collector{};  // Container for all relevan values
    ifstream filestream(paramDoc);  // File stream

    if (filestream.is_open()) {
      while (std::getline(filestream, line)) {
        istringstream linestream(line);
        while (linestream >> key) {
          if (key == targetKey) {
            // Extract relevant values
            while (linestream >> value) collector.push_back(value);
            return collector;
          }
        }
      }
    } else {
      std::cerr << "cannot find parameter files." << std::endl;
    }

    return collector;  // Empty vector
  }

};

#endif
