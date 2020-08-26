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

#include <gmock/gmock.h>
#include "rosbag_mock_drivers/BagParser.h"
#include <carma_simulation_msgs/BagData.h>


namespace mock_drivers{

    TEST(BagParser, Constructor){
        BagParser d1("");
        BagParser d2("", false);
        BagParser d3("", true);

        ASSERT_EQ(d1.getMockDriverNode().isDummy(), false);
        ASSERT_EQ(d2.getMockDriverNode().isDummy(), false);
        ASSERT_EQ(d3.getMockDriverNode().isDummy(), true);
    }

    TEST(BagParser, publishCallback){
        BagParser d("resource/hello_test_bag.bag", true);

        EXPECT_TRUE(d.publishCallback());
    }

    TEST(BagParser, run){
        BagParser d("", true);
        ASSERT_EQ(d.run(), 0);
    }

}