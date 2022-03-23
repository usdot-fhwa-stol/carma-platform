#pragma once

/*
 * Copyright (C) <SUB><year> LEIDOS.
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

#include <iostream>
#include <vector>

namespace <SUB><package_name>
{

  /**
   * \brief Stuct containing the algorithm configuration values for <SUB><package_name>
   */
  struct Config
  {
    //! Example parameter
    std::string example_param = "My example param";

    // Stream operator for this config
    // TODO for USER: Update prints for the added parameters
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "<SUB><package_name>::Config { " << std::endl
           << "example_param: " << c.example_param << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // <SUB><package_name>