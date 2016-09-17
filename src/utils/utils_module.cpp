#include <stdio.h>
#include <math.h>
#include <list>
#include <numeric>

#include "utils_module.h"

std::string path_to_js(std::vector< std::vector< std::vector<float> > > path){
  char buf[50];
  std::string c_string("[");
  for (size_t layer = 0; layer < path.size(); layer += 1){
    c_string += "[";
      for (size_t point = 0; point < path[layer].size(); point += 1){
        // sprintf(buf, "[%g,%g,%g,%d]", path[layer][point][0], path[layer][point][1], path[layer][point][2], (int)path[layer][point][3]);
        sprintf(buf, "[%.2f,%.2f,%.2f,%d]", path[layer][point][0], path[layer][point][1], path[layer][point][2], (int)path[layer][point][3]);
        c_string += buf;
        c_string += ",";
      }
      c_string.erase(c_string.end() - 1);
    c_string += "],";
  }
  c_string.erase(c_string.end() - 1);
  c_string += "]";
  return c_string;
}


// Subset of http://msdn.microsoft.com/en-us/library/system.text.stringbuilder.aspx
template <typename chr>
class StringBuilder {
    typedef std::basic_string<chr> string_t;
    typedef std::list<string_t> container_t; // Reasons not to use vector below.
    typedef typename string_t::size_type size_type; // Reuse the size type in the string.
    container_t m_Data;
    size_type m_totalSize;
    void append(const string_t &src) {
        m_Data.push_back(src);
        m_totalSize += src.size();
    }
    // No copy constructor, no assignement.
    StringBuilder(const StringBuilder &);
    StringBuilder & operator = (const StringBuilder &);
public:
    StringBuilder(const string_t &src) {
        if (!src.empty()) {
            m_Data.push_back(src);
        }
        m_totalSize = src.size();
    }
    StringBuilder() {
        m_totalSize = 0;
    }
    // TODO: Constructor that takes an array of strings.
 
 
    StringBuilder & Append(const string_t &src) {
        append(src);
        return *this; // allow chaining.
    }
        // This one lets you add any STL container to the string builder.
    template<class inputIterator>
    StringBuilder & Add(const inputIterator &first, const inputIterator &afterLast) {
        // std::for_each and a lambda look like overkill here.
                // <b>Not</b> using std::copy, since we want to update m_totalSize too.
        for (inputIterator f = first; f != afterLast; ++f) {
            append(*f);
        }
        return *this; // allow chaining.
    }
    // TODO: AppendFormat implementation. Not relevant for the article.
 
    // Like C# StringBuilder.ToString()
    // Note the use of reserve() to avoid reallocations.
    string_t ToString() const {
        string_t result;
        // The whole point of the exercise!
        // If the container has a lot of strings, reallocation (each time the result grows) will take a serious toll,
        // both in performance and chances of failure.
        // I measured (in code I cannot publish) fractions of a second using 'reserve', and almost two minutes using +=.
        result.reserve(m_totalSize + 1);
    //  result = std::accumulate(m_Data.begin(), m_Data.end(), result); // This would lose the advantage of 'reserve'
        for (auto iter = m_Data.begin(); iter != m_Data.end(); ++iter) {
            result += *iter;
        }
        return result;
    }
 
    // like javascript Array.join()
    string_t Join(const string_t &delim) const {
        if (delim.empty()) {
            return ToString();
        }
        string_t result;
        if (m_Data.empty()) {
            return result;
        }
        // Hope we don't overflow the size type.
        size_type st = (delim.size() * (m_Data.size() - 1)) + m_totalSize + 1;
        result.reserve(st);
                // If you need reasons to love C++11, here is one.
        struct adder {
            string_t m_Joiner;
            adder(const string_t &s): m_Joiner(s) {
                // This constructor is NOT empty.
            }
                        // This functor runs under accumulate() without reallocations, if 'l' has reserved enough memory.
            string_t operator()(string_t &l, const string_t &r) {
                l += m_Joiner;
                l += r;
                return l;
            }
        } adr(delim);
        auto iter = m_Data.begin();
                // Skip the delimiter before the first element in the container.
        result += *iter;
        return std::accumulate(++iter, m_Data.end(), result, adr);
    }
 
}; // class StringBuilder

std::string path_to_js_cpp(std::vector< std::vector< PathVector > > output){
  char buf[50];
  StringBuilder<char> sb;
  sb.Append("[");
  for (size_t layer = 0; layer < output.size(); layer += 1){
    sb.Append("[");
    for (size_t point = 0; point <output[layer].size(); point += 1){
      // sprintf(buf, "[%g,%g,%g,%d]",output[layer][point][0],output[layer][point][1],output[layer][point][2], (int)(*fc->native_path)[layer][point][3]);
      sprintf(buf, "[%.2f,%.2f,%.2f,%d]",output[layer][point].x,output[layer][point].y,output[layer][point].z,output[layer][point].type);
      sb.Append(buf);
      if(point !=output[layer].size()-1) sb.Append(",");
    }
    if(layer != output.size()-1){
      sb.Append("],");
    }else{
      sb.Append("]");
    }
  }
  sb.Append("]");
  return sb.ToString();
}
