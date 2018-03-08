/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.mobilityrouter;

import java.util.List;

// Java implementation of search and insert operations
// based off https://www.geeksforgeeks.org/trie-insert-and-search/
// on Trie
public class CallbackTrie {
     
  // Alphabet size (# of symbols)
  static final int ALPHABET_SIZE = 27;
   
  // trie node
  static class TrieNode
  {
      TrieNode[] children = new TrieNode[ALPHABET_SIZE];
      List<Object> mobiltyRequestHandlers;
      List<Object> mobiltyAckHandlers;
      List<Object> mobiltyOperationHandlers;
      List<Object> mobiltyStatusHandlers;
      boolean isEndOfCapability;
       
      TrieNode(){
        isEndOfCapability = false;
          for (int i = 0; i < ALPHABET_SIZE; i++)
              children[i] = null;
      }
  };
    
  static TrieNode root; 
   
  // If not present, inserts key into trie
  // If the key is prefix of trie node, 
  // just marks leaf node
  static void insert(String key)
  {
      int level;
      int length = key.length();
      int index;
    
      TrieNode pCrawl = root;
    
      for (level = 0; level < length; level++)
      {
          index = key.charAt(level) - 'a';
          if (pCrawl.children[index] == null)
              pCrawl.children[index] = new TrieNode();
    
          pCrawl = pCrawl.children[index];
      }
    
      // mark last node as leaf
      pCrawl.isEndOfCapability = true;
  }
    
  // Returns true if key presents in trie, else false
  private static TrieNode search(String key)
  {
      int level;
      int length = key.length();
      int index;
      TrieNode pCrawl = root;
    
      for (level = 0; level < length; level++)
      {
          index = key.charAt(level) - 'a';
    
          if (pCrawl.children[index] == null)
              return null;
    
          pCrawl = pCrawl.children[index];
      }
    
      // TODO think about what to return here
      //return (pCrawl != null && pCrawl.isEndOfCapability);
      return null;
  }