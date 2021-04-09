// __ ___ ____ _____ ______ _______ ________ _______ ______ _____ ____ ___ __
//
// Copyright (C) 2012-2016, Fabian Schmidt <crocdialer@googlemail.com>
//
// It is distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
// __ ___ ____ _____ ______ _______ ________ _______ ______ _____ ____ ___ __

//  Timer.cpp
//
//  Created by Croc Dialer on 27/07/14.

#pragma once

#include "Timer.hpp"
#include "Arduino.h"

namespace kinski
{

Timer::Timer():
m_interval(0),
m_start_time(0),
m_periodic(false),
m_timer_cb(nullptr)
{

}

void Timer::poll()
{
    // expired
    if(m_start_time && millis() >= m_start_time + m_interval)
    {
        m_start_time = m_periodic ? millis() : 0;
        if(m_timer_cb){ m_timer_cb(); }
    }
}

void Timer::expires_from_now(Timer::Real secs)
{
    m_start_time = millis();
    m_interval = secs * 1000;
}

Timer::Real Timer::expires_from_now() const
{
    return (m_start_time + m_interval - millis()) / Real(1000);
}

bool Timer::has_expired() const
{
    return millis() > m_start_time + m_interval;
}

void Timer::cancel()
{
    m_start_time = 0;
}

bool Timer::periodic() const
{
    return m_periodic;
}

void Timer::set_periodic(bool b)
{
    m_periodic = b;
}

void Timer::set_callback(timer_cb_t cb)
{
    m_timer_cb = cb;
}

}// namespace
