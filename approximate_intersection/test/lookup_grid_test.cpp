/*
 * Copyright (C) 2022 LEIDOS.
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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "approximate_intersection/lookup_grid.hpp"

namespace approximate_intersection {

struct TestPoint {
    double x = 0;
    double y = 0;
};

TEST(approximate_intersection, test){

    Config config;
    config.min_x = -10;
    config.max_x = 10;
    config.min_y = -10;
    config.max_y = 10;
    config.cell_side_length = 1;

    LookupGrid<TestPoint> grid(config);

    // Verify no intersections for any point
    for (double i = -9.5; i < 10; i += 1.0) {
        for (double j = -9.5; j < 10; j += 1.0) {
            TestPoint p;
            p.x = i;
            p.y = j;
            ASSERT_FALSE(grid.intersects(p));
        }
    }

    // Add some points on the diagonal
    for (double i = -9.5; i < 10; i += 1.0) {
        TestPoint p;
        p.x = i;
        p.y = i;
        grid.insert(p);
    }

    // Verify intersection
    for (double i = -9.5; i < 10; i += 1.0) {
        for (double j = -9.5; j < 10; j += 1.0) {
            TestPoint p;
            p.x = i;
            p.y = j;
            if (i == j) {
                ASSERT_TRUE(grid.intersects(p));
            } else {
                ASSERT_FALSE(grid.intersects(p));
            }
        }
    }
    

}

} // approximate_intersection

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    bool success = RUN_ALL_TESTS();

    return success;
} 