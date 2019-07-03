#!/bin/bash

#  Copyright (C) 2018-2019 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

cd "$(dirname "$0")"
REPO_NAME="$(./get-repo-name.sh)"

# This sed command based on Stack Overflow answer: https://stackoverflow.com/questions/28795479/awk-sed-script-to-convert-a-file-from-camelcase-to-underscores
# Asked by Corentin Peuvrel: https://stackoverflow.com/users/4608146/corentin-peuvrel
# Answered by pachopepe: https://stackoverflow.com/users/641896/pachopepe
# Credited in accordance with Stack Overflow's CC-BY license
echo $REPO_NAME | sed -r 's/CARMA/carma/' | sed -r 's/([A-Z])/-\L\1/g' | sed 's/^_//'

