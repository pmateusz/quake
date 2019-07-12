//
// Copyright 2018 Mateusz Polnik
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QUAKE_MINIZINC_WRITER_H
#define QUAKE_MINIZINC_WRITER_H

#include <iomanip>
#include <ostream>
#include <string>
#include <utility>

namespace quake {

    template<typename OutputStreamType>
    class MiniZincWriter {
    public:
        explicit MiniZincWriter(OutputStreamType &&stream) noexcept;

        MiniZincWriter(const MiniZincWriter<OutputStreamType> &other) = delete;

        MiniZincWriter<OutputStreamType> &operator=(const MiniZincWriter<OutputStreamType> &other) = delete;

        MiniZincWriter(MiniZincWriter<OutputStreamType> &&other) noexcept;

        MiniZincWriter<OutputStreamType> &operator=(MiniZincWriter<OutputStreamType> &&other) noexcept;

        template<typename ValueType>
        std::string Quote(const ValueType &value) const;

        template<typename NumericType>
        void WriteVariable(const std::string &name, const NumericType &value);

        template<typename IteratorType>
        void WriteEnum(const std::string &name, IteratorType begin_it, IteratorType end_it);

        template<typename NumericType1, typename NumericType2>
        void WriteRange(const std::string &name, NumericType1 from, NumericType2 to);

        template<typename IteratorType>
        void WriteArray1d(const std::string &name, IteratorType begin_it, IteratorType end_it);

        template<typename IteratorType>
        void WriteArray2d(const std::string &name, IteratorType begin_it, IteratorType end_it);

        void WriteNewLine();

        void Close();

        virtual ~MiniZincWriter();

    private:
        OutputStreamType stream_;

        void TryClose();

        template<typename IteratorType>
        void WriteArrayContent1d(IteratorType begin_it, IteratorType end_it, bool numbers_padded);

        template<typename NumberType>
        void WriteNumber(NumberType value, bool padded);
    };
}

namespace quake {

    template<typename OutputStreamType>
    MiniZincWriter<OutputStreamType>::MiniZincWriter(OutputStreamType &&stream) noexcept
            : stream_{std::move(stream)} {}

    template<typename OutputStreamType>
    void MiniZincWriter<OutputStreamType>::Close() {
        stream_.close();
    }

    template<typename OutputStreamType>
    MiniZincWriter<OutputStreamType>::~MiniZincWriter() {
        TryClose();
    }

    template<typename OutputStreamType>
    MiniZincWriter<OutputStreamType>::MiniZincWriter(MiniZincWriter<OutputStreamType> &&other) noexcept {
        TryClose();

        stream_ = std::move(other.stream_);
        other.stream_ = {};
    }

    template<typename OutputStreamType>
    MiniZincWriter<OutputStreamType> &
    MiniZincWriter<OutputStreamType>::operator=(MiniZincWriter<OutputStreamType> &&other) noexcept {
        TryClose();

        stream_ = std::move(other.stream_);
        other.stream_ = {};

        return *this;
    }

    template<typename OutputStreamType>
    void MiniZincWriter<OutputStreamType>::TryClose() {
        if (stream_.is_open()) {
            stream_.close();
        }
    }

    template<typename OutputStreamType>
    template<typename NumericType>
    void MiniZincWriter<OutputStreamType>::WriteVariable(const std::string &name, const NumericType &value) {
        stream_ << name << " = " << value << ";" << std::endl;
    }

    template<typename OutputStreamType>
    template<typename IteratorType>
    void MiniZincWriter<OutputStreamType>::WriteEnum(const std::string &name,
                                                     IteratorType begin_it,
                                                     IteratorType end_it) {
        stream_ << name << " = { ";

        auto current_it = begin_it;
        if (current_it != end_it) {
            stream_ << *current_it;
            ++current_it;
        }

        for (; current_it != end_it; ++current_it) {
            stream_ << ", " << *current_it;
        }

        stream_ << " };" << std::endl;
    }

    template<typename OutputStreamType>
    template<typename NumericType1, typename NumericType2>
    void MiniZincWriter<OutputStreamType>::WriteRange(const std::string &name, NumericType1 from, NumericType2 to) {
        stream_ << name << " = " << from << ".." << to << ';' << std::endl;
    }

    template<typename OutputStreamType>
    template<typename IteratorType>
    void MiniZincWriter<OutputStreamType>::WriteArray1d(const std::string &name, IteratorType begin_it,
                                                        IteratorType end_it) {
        static const auto NUMBERS_PADDED = false;

        stream_ << name << " = [";
        WriteArrayContent1d(begin_it, end_it, NUMBERS_PADDED);
        stream_ << "];" << std::endl;
    }

    template<typename OutputStreamType>
    template<typename IteratorType>
    void MiniZincWriter<OutputStreamType>::WriteArray2d(const std::string &name, IteratorType begin_it,
                                                        IteratorType end_it) {
        static const auto NUMBERS_PADDED = true;

        static const std::string ASSIGNMENT_PART{" = [| "};

        stream_ << name << ASSIGNMENT_PART;

        const auto offset_size = name.length() + ASSIGNMENT_PART.length() - 2;

        auto current_row_it = begin_it;
        if (current_row_it != end_it) {
            WriteArrayContent1d(std::cbegin(*current_row_it), std::cend(*current_row_it), NUMBERS_PADDED);
            ++current_row_it;
        }

        for (; current_row_it != end_it; ++current_row_it) {
            stream_ << std::endl;

            for (auto i = 0; i < offset_size; ++i) {
                stream_ << ' ';
            }

            stream_ << "| ";
            WriteArrayContent1d(std::cbegin(*current_row_it), std::cend(*current_row_it), NUMBERS_PADDED);
        }

        stream_ << " |];" << std::endl;
    }

    template<typename OutputStreamType>
    template<typename NumberType>
    void MiniZincWriter<OutputStreamType>::WriteNumber(NumberType value, bool padded) {
        if (padded) {
            stream_ << std::setfill(' ') << std::setw(7) << std::setprecision(4) << std::fixed;
        }

        stream_ << value;
    }

    template<typename OutputStreamType>
    template<typename IteratorType>
    void MiniZincWriter<OutputStreamType>::WriteArrayContent1d(IteratorType begin_it,
                                                               IteratorType end_it,
                                                               bool numbers_padded) {
        auto current_it = begin_it;
        if (current_it != end_it) {
            WriteNumber(*current_it, numbers_padded);
            ++current_it;
        }

        for (; current_it != end_it; ++current_it) {
            stream_ << ", ";
            WriteNumber(*current_it, numbers_padded);
        }
    }

    template<typename OutputStreamType>
    template<typename ValueType>
    std::string MiniZincWriter<OutputStreamType>::Quote(const ValueType &value) const {
        std::stringstream output;
        output << "\"" << value << "\"";
        return output.str();
    }

    template<typename OutputStreamType>
    void MiniZincWriter<OutputStreamType>::WriteNewLine() {
        stream_ << std::endl;
    }
}

#endif //QUAKE_MINIZINC_WRITER_H
