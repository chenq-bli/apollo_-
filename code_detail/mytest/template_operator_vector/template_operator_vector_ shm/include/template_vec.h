#pragma once
#include <iostream>
#include <string>

namespace MySTL
{
    using namespace std;
    template <typename T>
    class Vector
    {
    public:
        typedef T value_type;
        typedef T *iterator;

    public:
        Vector() : date_(nullptr), size_(0), capacity_(0)
        {
        }
        Vector(int size, value_type v)
        {
            size_ = size;
            capacity_ = size;
            date_ = new value_type[capacity_];
            for (int i = 0; i < size_; i++)
            {
                date_[i] = v;
            }
        }
        Vector(const Vector &vec)
        {
            this->size_ = vec.size_;
            this->capacity_ = vec.capacity_;
            date_ = new value_type[capacity_];
            for (int i = 0; i < size_; i++)
            {
                date_[i] = vec.date_[i];
            }
        }
        ~Vector()
        {
            delete[] date_;
            date_ = nullptr;
            size_ = 0;
            capacity_ = 0;
        }

        Vector &operator=(const Vector &vec)
        {
            if (this == &vec)
                return *this;
            value_type *temp = new value_type[vec.capacity_];
            this->capacity_ = vec.capacity_;
            this->size_ = vec.size_;
            for (int i = 0; i < size_; ++i)
            {
                temp[i] = vec.date_[i];
            }
            delete[] date_;
            date_ = temp;
            return *this;
        }

        void push_back(value_type value)
        {
            if (capacity_ == 0)
            {
                capacity_ = 1;
                date_ = new value_type[1];
            }
            else if (size_ + 1 > capacity_)
            {
                capacity_ = 2 * capacity_;
                value_type *temp = new value_type[capacity_];
                for (int i = 0; i < size_; i++)
                {
                    temp[i] = date_[i];
                }
                delete[] date_;
                date_ = temp;
            }
            date_[size_] = value;
            size_++;
        }
        void pop_back()
        {
            size_--;
        }
        int size()
        {
            return size_;
        }

        int capacity()
        {
            return capacity_;
        }

        bool empty()
        {
            return size_ == 0;
        }

        void erase(iterator it)
        {
            int index = it - date_;
            for (int i = index; i < size_ - 1; i++)
            {
                date_[i] = date_[i + 1];
            }
            --size_;
        }

    public:
        value_type &operator[](int index)
        {
            if (index > size_ - 1)
            {
                cout << "访问失败！" << endl;
            }
            else
            {
                return date_[index];
            }
        }
        value_type front()
        {
            return date_[0];
        }
        value_type back()
        {
            return date_[size_ - 1];
        }
        iterator begin()
        {
            return date_;
        }
        iterator end()
        {
            return date_ + size_;
        }

    private:
        value_type *date_;
        int size_;
        int capacity_;

    private:
    };

}