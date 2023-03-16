// bufferAdaptor.h
//

// +++DRAFT+++ This class implements the conversion from std array to std vector without copy
// More precisely, it defines an object with vector characteristics, that can be initialized from an array without copy
// It is templated for both basic double and also the extended types used for automatic differentiation


#pragma once

#include <cstddef>
#include <iterator>
#include <stdexcept>

namespace bcp {

    template <typename T>
    class buffer_adaptor
    {
    public:
        using self_type = buffer_adaptor<T>;
        using size_type = std::size_t;
        using value_type = T;
        using pointer = T*;
        using const_pointer = const T*;
        using reference = value_type&;
        using const_reference = const value_type&;
        using difference_type = std::ptrdiff_t;

        using iterator = pointer;
        using const_iterator = const_pointer;
        using reverse_iterator = std::reverse_iterator<iterator>;
        using const_reverse_iterator = std::reverse_iterator<const_iterator>;

        explicit buffer_adaptor(size_type n, T *);
        explicit buffer_adaptor(size_type n, const T *);

        ~buffer_adaptor(void);

        reference operator[](size_type idx);
        const_reference operator[](size_type idx) const;

        reference at(size_type idx);
        const_reference at(size_type idx) const;

        pointer data(void);
        const_pointer data(void) const;

        iterator begin(void);
        const_iterator begin(void) const;
        const_iterator cbegin(void) const;
        iterator end(void);
        const_iterator end(void) const;
        const_iterator cend(void) const;

        reverse_iterator rbegin(void);
        const_reverse_iterator rbegin(void) const;
        const_reverse_iterator crbegin(void) const;
        reverse_iterator rend(void);
        const_reverse_iterator rend(void) const;
        const_reverse_iterator crend(void) const;

        bool empty(void) const;
        size_type size(void) const;

        reference front(void);
        const_reference front(void) const;
        reference back(void);
        const_reference back(void) const;

    private:
        size_type m_size = 0;
        pointer m_begin = nullptr;
        pointer m_end = nullptr;
    };

    template <typename T>
    inline buffer_adaptor<T>::buffer_adaptor(size_type n, T *t) : m_size(n), m_begin(t), m_end(t+n)
    {}

    template <typename T>
    inline buffer_adaptor<T>::buffer_adaptor(size_type n, const T *t) : m_size(n), m_begin(const_cast<T*>(t)), m_end(const_cast<T *>(t)+n)
    {}

    template <typename T>
    inline buffer_adaptor<T>::~buffer_adaptor(void)
    {
        m_size = 0;
        m_begin = nullptr;
        m_end = nullptr;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::operator[](size_type i) -> reference
    {
        return m_begin[i];
    }

    template <typename T>
    inline auto buffer_adaptor<T>::operator[](size_type i) const ->const_reference
    {
        return m_begin[i];
    }

    template <typename T>
    inline auto buffer_adaptor<T>::at(size_type i) -> reference
    {
        if (i >= m_size) {
            throw std::out_of_range("Out of range in array access");
        }
        return this->operator[](i);
    }

    template <typename T>
    inline auto buffer_adaptor<T>::at(size_type i) const -> const_reference
    {
        if (i >= m_size) {
            throw std::out_of_range("Out of range in array access");
        }
        return this->operator[](i);
    }

    template <typename T>
    inline auto buffer_adaptor<T>::data(void) -> pointer
    {
        return m_begin;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::data(void) const -> const_pointer
    {
        return m_begin;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::begin(void) -> iterator
    {
        return m_begin;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::begin(void) const -> const_iterator
    {
        return m_begin;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::cbegin(void) const -> const_iterator
    {
        return m_begin;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::end(void) -> iterator
    {
        return m_end;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::end(void) const -> const_iterator
    {
        return m_end;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::cend(void) const -> const_iterator
    {
        return m_end;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::rbegin(void) -> reverse_iterator
    {
        return reverse_iterator(end());
    }

    template <typename T>
    inline auto buffer_adaptor<T>::rbegin(void) const -> const_reverse_iterator
    {
        return const_reverse_iterator(end());
    }

    template <typename T>
    inline auto buffer_adaptor<T>::crbegin(void) const -> const_reverse_iterator
    {
        return rbegin();
    }

    template <typename T>
    inline auto buffer_adaptor<T>::rend(void) -> reverse_iterator
    {
        return reverse_iterator(begin());
    }

    template <typename T>
    inline auto buffer_adaptor<T>::rend(void) const ->const_reverse_iterator
    {
        return const_reverse_iterator(begin());
    }

    template <typename T>
    inline auto buffer_adaptor<T>::crend(void) const -> const_reverse_iterator
    {
        return rend();
    }

    template <typename T>
    inline auto buffer_adaptor<T>::empty(void) const -> bool
    {
        return m_begin == m_end;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::size(void) const -> size_type
    {
        return m_size ;
    }

    template <typename T>
    inline auto buffer_adaptor<T>::front(void) -> reference
    {
        return m_begin[0];
    }

    template <typename T>
    inline auto buffer_adaptor<T>::front(void) const -> const_reference
    {
        return m_begin[0];
    }

    template <typename T>
    inline auto buffer_adaptor<T>::back(void) -> reference
    {
        return *(m_end - 1);
    }

    template <typename T>
    inline auto buffer_adaptor<T>::back(void) const -> const_reference
    {
        return *(m_end - 1);
    }

}

//
// bufferAdaptor.h ends here
