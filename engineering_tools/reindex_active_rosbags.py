#!/usr/bin/python3

#  Copyright (C) 2021 LEIDOS.
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

import os
import subprocess
import sys

# Usage:
# python reindex_active_rosbags.py <path to folder containing .bag.active files>

def reindex_bag_files(source_folder):
    # List of bag files to re-index
    # Note: This example list includes .bag.active files from TIM use case testing 5/25-5/27
    files_to_reindex = ["_2021-05-25-17-47-01.bag.active", 
                        "_2021-05-25-17-53-47.bag.active", 
                        "_2021-05-25-18-14-12.bag.active", 
                        "_2021-05-25-18-19-06.bag.active", 
                        "_2021-05-25-18-24-50.bag.active", 
                        "_2021-05-25-18-54-32.bag.active", 
                        "_2021-05-25-19-06-02.bag.active", 
                        "_2021-05-25-19-19-45.bag.active",  
                        "_2021-05-25-20-32-22.bag.active", 	
                        "_2021-05-25-20-49-05.bag.active", 	
                        "_2021-05-26-13-09-39.bag.active",		
                        "_2021-05-26-13-33-10.bag.active",	
                        "_2021-05-26-17-26-06.bag.active",
                        "_2021-05-26-19-45-45.bag.active",
                        "_2021-05-27-20-34-31.bag.active"]
    
    # Logic to re-index bag files
    for filename in files_to_reindex:
        print("Reindexing " + str(filename))
        
        # Issue command to re-index the bag file
        # References example from https://github.com/bierschi/reindexBags
        command = "rosbag reindex " + str(filename)
        reindex_proc = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=source_folder,
                                         executable='/bin/bash')
        reindex_proc.wait()
        
        # If a .orig.active file was created, then the .bag.active file was re-indexed
        temp_file_name = str(filename[:-7]) + ".orig.active"
        if (temp_file_name in os.listdir(source_folder)):

            # Not removing the .bag.orig.active file for now, can delete manually afterwards in case the re-indexed file is corrupted 
            #os.remove(temp_file_name)

            # Rename the re-indexed .bag.active to '.bag' only
            os.rename(filename, filename[:-7])

            # Print finished statement
            print("Finished re-indexing " + str(filename))

    return

def main():  
    if len(sys.argv) < 2:
        print("Need 1 arguments: process_bag.py <path to source folder with .bag.active files> ")
        exit()
    
    source_folder = sys.argv[1]

    reindex_bag_files(source_folder)

    return

if __name__ == "__main__":
    main()