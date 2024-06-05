/*
 * Copyright (C) 2018-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <unordered_map>
#include <algorithm>
#include <mutex>
#include <iostream>

/**
 * Helper functions for recording calls without use of Mocks in unit tests. 
 * User must call markCalled(test name, function name) inside each function call
 * callCount can be used to get the number of times a function was called
 * clearHistory can be used to clear the history of a specific test
 * 
 * These functions are thread safe and support usage in multiple test cases
 * 
 * Functions should be referred to using the pattern Class::obj::function for example "A::a::getValue"
 * Free functions should be referred to without class and obj designations such as "::freeFunc"
 * These patterns help keep names more unique
 */ 
namespace CallRecorder {

  std::mutex map_mutex_; // Make recorder thread safe incase there is multi-threading of unit tests
  std::unordered_map<std::string, std::unordered_map<std::string, unsigned int>> callMap_;
  // Mark function as called
  void markCalled(const std::string& testKey, const std::string& fullFunctionName) { // Class::obj::function
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (callMap_.find(testKey) == callMap_.end()) {
      std::unordered_map<std::string, unsigned int> calls;
      callMap_[testKey] = calls;
    }
    auto test_map = callMap_[testKey];

    if (test_map.find(fullFunctionName) == test_map.end()) {
      test_map[fullFunctionName] = 0;
    }
    test_map[fullFunctionName] += 1;

    callMap_[testKey] = test_map;
  }
  // Get the call count of a function
  unsigned int callCount(const std::string& testKey, const std::string& fullFunctionName) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto test_calls_map = callMap_[testKey];
    if (test_calls_map.find(fullFunctionName) == test_calls_map.end()) {
      return 0;
    }
    return test_calls_map[fullFunctionName];
  }
  // Clear function call history for a specific test
  void clearHistory(const std::string& testKey) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    callMap_[testKey].clear();
    callMap_.erase(testKey);
  }
};