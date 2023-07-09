#pragma once

#include <shared_mutex>
#include <unordered_map>

namespace concurrent_collection
{
    template <typename Key, typename Value>
    /*!
     * A class that contains a thread-safe unordered_map.
     */
    class ConcurrentUnorderedMap
    {
    public:
        /*!
         * Inserts a key-value pair into the concurrent unordered_map.
         * @param key   The key to be inserted.
         * @param value The value to be associated with the key.
         */
        void insert(const Key& key, const Value& value)
        {
            std::unique_lock<std::shared_mutex> lock(mutex_);
            map_[key] = value;
        }

        /*!
         * Checks if the given key is present in the concurrent unordered_map.
         * @param key The key to be checked.
         * @return    True if the key is found, false otherwise.
         */
        bool contains(const Key& key)
        {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return map_.find(key) != map_.end();
        }

        /*!
         * Retrieves the value associated with the given key from the concurrent unordered_map.
         * @param key The key to retrieve the value for.
         * @return    The value associated with the key if found, or a default-constructed value otherwise.
         */
        Value get(const Key& key)
        {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return map_[key];
        }

        /*!
         * Returns the current size of the concurrent unordered_map.
         * @return The size of the map.
         */
        int size()
        {
            return map_.size();
        }

    private:
        std::unordered_map<Key, Value> map_;  // The underlying unordered_map.
        mutable std::shared_mutex mutex_;     // The mutex for concurrent access.
    };
}