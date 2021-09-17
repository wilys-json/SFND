#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <fstream>
#include <string>
#include <vector>
#include <iostream>

using std::string;
using std::ifstream;
using std::vector;
using std::istringstream;



namespace Parameters {

  const string paramDoc{"parameters.txt"};

  // Methods
  template <typename T>
  vector<T> getValues(const string& targetKey) {
    string line, key;
    T value;
    vector<T> collector{};
    ifstream filestream(paramDoc);
    std::cout << "is open: " << filestream.is_open() << std::endl;
      while (std::getline(filestream, line)) {
        istringstream linestream(line);
        std::cout << "line" << std::endl;
        while (linestream >> key) {
          if (key == targetKey) {
            while (linestream >> value) collector.push_back(value);
            return collector;
          }
        }
      }
    filestream.close();
    return collector;
  }

  template <typename T>
  vector<T> get(const string& targetKey) {
    return getValues<T>(targetKey);
  }

};

#endif
