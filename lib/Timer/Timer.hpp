// __ ___ ____ _____ ______ _______ ________ _______ ______ _____ ____ ___ __
//
// Copyright (C) 2012-2016, Fabian Schmidt <crocdialer@googlemail.com>
//
// It is distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
// __ ___ ____ _____ ______ _______ ________ _______ ______ _____ ____ ___ __

//  Timer.h
//
//  Created by Croc Dialer on 27/07/14.

#pragma once

#include <stdint.h>

namespace kinski
{
    class Timer
    {
    public:
        typedef void (*timer_cb_t)();

        using Real = float;

        Timer();

        /*!
         * manual polling
         */
        void poll();

        /*!
         * set expiration date from now in seconds
         */
        void expires_from_now(Real secs);

        /*!
         * get expiration date from now in seconds
         */
        Real expires_from_now() const;

        /*!
         * returns true if the timer has expired
         */
        bool has_expired() const;

        /*!
         * cancel a currently running timer
         */
        void cancel();

        /*!
         * returns true if the timer is set to fire periodically
         */
        bool periodic() const;

        /*!
         * sets if the timer should fire periodically
         */
        void set_periodic(bool b = true);

        /*!
         * set the function object to call when the timer expires
         */
        void set_callback(timer_cb_t cb = timer_cb_t());

    private:
        uint32_t m_interval, m_start_time;
        bool m_periodic;
        timer_cb_t m_timer_cb;
    };
}
