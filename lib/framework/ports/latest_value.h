#ifndef LATEST_VALUE_H
#define LATEST_VALUE_H

#include <Arduino.h>

namespace port
{
    template<typename value_type>
    class latest_value;

    template<typename value_type>
    class latest_reader
    {
        public:
            latest_reader() = default;
            explicit latest_reader(latest_value<value_type> *channel)
                : channel(channel)
            {
            }

        public:
            bool read(value_type &out) const
            {
                return channel && channel->read(out);
            }

            bool connected() const
            {
                return channel != nullptr;
            }

        private:
            latest_value<value_type> *channel = nullptr;
    };

    template<typename value_type>
    class latest_writer
    {
        public:
            latest_writer() = default;
            explicit latest_writer(latest_value<value_type> *channel)
                : channel(channel)
            {
            }

        public:
            bool publish(const value_type &value) const
            {
                if(!channel){return false;}
                channel->publish(value);
                return true;
            }

            bool connected() const
            {
                return channel != nullptr;
            }

        private:
            latest_value<value_type> *channel = nullptr;
    };

    template<typename value_type>
    class latest_value
    {
        public:
            latest_value() = default;

        public:
            latest_reader<value_type> reader()
            {
                return latest_reader<value_type>(this);
            }

            latest_writer<value_type> writer()
            {
                return latest_writer<value_type>(this);
            }

        private:
            friend class latest_reader<value_type>;
            friend class latest_writer<value_type>;

            bool read(value_type &out)
            {
                bool available = false;
                portENTER_CRITICAL(&lock);
                available = valid;
                if(available){out = value;}
                portEXIT_CRITICAL(&lock);
                return available;
            }

            void publish(const value_type &next)
            {
                portENTER_CRITICAL(&lock);
                value = next;
                valid = true;
                portEXIT_CRITICAL(&lock);
            }

        private:
            value_type value{};
            portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
            bool valid = false;
    };
}

#endif
