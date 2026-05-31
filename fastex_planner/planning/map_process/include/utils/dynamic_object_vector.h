/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-26 12:23:11
 * @LastEditTime: 2025-10-28 11:17:34
 * @Description:
 */
#ifndef _DYNAMIC_OBJECT_VECTOR_
#define _DYNAMIC_OBJECT_VECTOR_

#include <stack>
#include <unordered_map>

/**
 * @brief The class to store the dynamic object vector
 *
 * @tparam T
 */
template <typename T> class DynamicObjectVector
{
  public:
    DynamicObjectVector() {};
    ~DynamicObjectVector() {};

    /**
     * @brief Set the object at the index
     *
     * @param index The index of the object
     * @param object The object to be set
     */
    void set(const unsigned int& index, const T& object)
    {
        dynamic_objects_vector_[index] = object;
    }

    /**
     * @brief Push the object into the vector
     *
     * @param object The object to be pushed
     * @return unsigned int The index of the object
     */
    unsigned int push(const T& object)
    {
        unsigned int free_index = getFreeIndex();
        dynamic_objects_vector_[free_index] = object;
        return free_index;
    }

    /**
     * @brief Erase the object at the index
     *
     * @param index The index of the object to be erased
     */
    size_t erase(const unsigned int& index)
    {
        free_indices_.push(index);
        return dynamic_objects_vector_.erase(index);
    }

    /**
     * @brief Erase the object at the iterator
     *
     * @param it The iterator of the object to be erased
     * @return std::unordered_map<unsigned int, T>::iterator
     */
    typename std::unordered_map<unsigned int, T>::iterator
    erase(typename std::unordered_map<unsigned int, T>::iterator it)
    {
        free_indices_.push(it->first);
        return dynamic_objects_vector_.erase(it);
    }

    /**
     * @brief Get the object at the index
     *
     * @param index The index of the object
     * @return T&
     */
    T& at(const unsigned int& index) { return dynamic_objects_vector_.at(index); }

    /**
     * @brief Get the object at the index
     *
     * @param index The index of the object
     * @return T
     */
    T at(const unsigned int& index) const { return dynamic_objects_vector_.at(index); }

    /**
     * @brief Clear the vector
     *
     */
    void clear()
    {
        dynamic_objects_vector_.clear();
        while (!free_indices_.empty())
            free_indices_.pop();
    }

    /**
     * @brief Get the size of the vector
     *
     * @return unsigned int The size of the vector
     */
    unsigned int size() const { return dynamic_objects_vector_.size(); }

    /**
     * @brief Get the object at the index
     *
     * @param index The index of the object
     * @return T The object at the index
     */
    T operator[](const unsigned int& index) const { return dynamic_objects_vector_.at(index); }

    /**
     * @brief Get the object at the index
     *
     * @param index The index of the object
     * @return T& The object at the index
     */
    T& operator[](const unsigned int& index) { return dynamic_objects_vector_.at(index); }

    auto begin() { return dynamic_objects_vector_.begin(); }
    auto end() { return dynamic_objects_vector_.end(); }
    auto begin() const { return dynamic_objects_vector_.cbegin(); }
    auto end() const { return dynamic_objects_vector_.cend(); }

  private:
    /**
     * @brief Get the free index
     *
     * @return unsigned int The free index
     */
    unsigned int getFreeIndex()
    {
        if (free_indices_.empty())
        {
            dynamic_objects_vector_[dynamic_objects_vector_.size()] = T();
            return dynamic_objects_vector_.size() - 1;
        }
        else
        {
            unsigned int index = free_indices_.top();
            free_indices_.pop();
            return index;
        }
    }

    // The vector to store the dynamic objects
    std::unordered_map<unsigned int, T> dynamic_objects_vector_;
    // The stack to store the free indices
    std::stack<unsigned int> free_indices_;
};

#endif // !_DYNAMIC_OBJECT_VECTOR_
