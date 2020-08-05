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
#pragma once

namespace mock_drivers{

    template <typename M, typename T>
    ROSComms<M, T>::ROSComms(std::function<void(T)> cbf, CommTypes ct, bool latch, int qs, std::string t){
        callback_function_ = cbf;
        comm_type_ = ct;
        latch_ = latch;
        queue_size_ = qs;
        topic_ = t;
    }

    template <typename T>
    ROSComms<T>::ROSComms(CommTypes ct, bool latch, int qs, std::string t){
        comm_type_ = ct;
        latch_ = latch;
        queue_size_ = qs;
        topic_ = t;
    };

    template <typename T>
    ROSComms<T>::ROSComms(std::function<void(T)> cbf, CommTypes ct, bool latch, int qs, std::string t){
        callback_function_ = cbf;
        comm_type_ = ct;
        latch_ = latch;
        queue_size_ = qs;
        topic_ = t;
    };
    
    template <typename M, typename T>
    M ROSComms<M, T>::getMessageType(){
        M element;
        return element;
    }

    template <typename M, typename T>
    T ROSComms<M, T>::getParamType(){
        T element;
        return element;
    }

    template <typename M>
    M ROSComms<M>::getTemplateType(){
        M element;
        return element;
    }

    template <typename M, typename T>
    void ROSComms<M, T>::callback(T msg){
        // call the callback_function_ function
        this->callback_function_(msg);
    }

    template <typename T>
    void ROSComms<T>::callback(T msg){
        // call the callback_function_ function
        this->callback_function_(msg);
    }



    
}