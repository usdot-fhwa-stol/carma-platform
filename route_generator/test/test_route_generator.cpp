/*
 * Copyright (C) 2019-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
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

#include "route_generator.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(RouteGeneratorTest, testReadFileFunction)
{
    std::vector<std::string> file_names = RouteGenerator::read_route_names("../resource/");
    sort(file_names.begin(),file_names.end()); // Ensure names are in alphabetical order
    for(size_t i = 0; i < file_names.size(); ++i)
    {
        std::string expect_file_name = "route" + std::to_string(i + 1) + ".csv";
        EXPECT_EQ(file_names[i], expect_file_name);
    }

}
TEST(RouteGeneratorTest, testRouteGenerator)
{
    RouteGenerator rg;
    EXPECT_FALSE(rg.is_route_active());
}
// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}