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
    typedef T* iterator;

    public:
    Vector();
    Vector(int size, value_type v);
    Vector(const Vector& vec);
    ~Vector();
    Vector& operator=(const Vector& vec);
    void push_back(value_type value);
    void pop_back();
    int size();
    int capacity();
    bool empty();
    void erase(iterator it);

    value_type& at(int index)
    {
        if(index>size_-1)
        {
            cout<<"访问失败！"<<endl;
        }
        else{
            return date_[index];  
        }
    }

    public:
    value_type& operator[](int index)
    {
        if(index>size_-1)
        {
            cout<<"访问失败！"<<endl;
        }
        else{
            return date_[index];  
        }
             
    }
    value_type front()
    {
        return date_[0]; 
    }
    value_type back()
    {
        return date_[size_-1]; 
    }
    iterator begin()
    {
        return date_;
    }
    iterator end()
    {
        return date_+size_;
    }

    private:
    value_type* date_;
    int size_;
    int capacity_;

    private:


    };

//实现最后不用放在CPP文件里面，我试过，会出现链接错误，需要显示实例化
//放在这个头文件就可以

template<typename T>
Vector<T>::Vector() : date_(nullptr),size_(0),capacity_(0)
{

}
template<typename T>
Vector<T>::Vector(const Vector& vec)
{
    this->size_=vec.size_;
    this->capacity_=vec.capacity_;
    date_=new value_type[capacity_];
    for(int i=0; i<size_ ; i++)
    {
        date_[i]=vec.date_[i];
    }

}

template<typename T>
Vector<T>::Vector(int size, value_type v)
{
    size_=size;
    capacity_=size;
    date_=new value_type[capacity_];
    for(int i=0; i<size_ ; i++)
    {
        date_[i]=v;
    }
}

template<typename T>
Vector<T>::~Vector()
{
    delete[] date_;
    date_=nullptr;
    size_=0;
    capacity_=0;
}

template<typename T>
Vector<T>& Vector<T>::operator=(const Vector& vec)
{
    if(this == &vec) return *this;
    value_type* temp= new value_type[vec.capacity_];
    this->capacity_=vec.capacity_;
    this->size_=vec.size_;
    for(int i=0; i<size_ ; ++i)
    {
        temp[i]=vec.date_[i];
    }
    delete[] date_;
    date_=temp;
    return *this;
    
}

template<typename T>
void Vector<T>::push_back(value_type value)
{
    if(capacity_==0)
    {
        capacity_=1;
        date_=new value_type[1];
    }
    else if(size_+1 > capacity_)
    {
        capacity_=2*capacity_;
        value_type* temp =new value_type[capacity_];
        for(int i=0; i<size_; i++)
        {
            temp[i]=date_[i];
        }
        delete[] date_;
        date_=temp;

    }
    date_[size_]=value;
    size_++;

}


template<typename T>
void Vector<T>::pop_back()
{
    size_--;
}

template<typename T>
int Vector<T>::size()
{
    return size_;
}

template<typename T>
int Vector<T>::capacity()
{
    return capacity_;
}

template<typename T>
bool Vector<T>::empty()
{
    return size_==0;
}

template<typename T>
void Vector<T>::erase(iterator it)
{
    int index= it-date_;
    for(int i=index; i<size_-1; i++)
    {
        date_[i]=date_[i+1];
    }
    --size_;
}





}