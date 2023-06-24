#include <iostream>
#include <boost/regex.hpp>
int main()
{
    std::string input = "Hello World me and phone fs3!%kk";
    boost::regex pattern("(\\w+)\\sW(\\w+)\\s(\\w+)(.*)%(\\w+)");
    boost::smatch result;
    if (boost::regex_match(input, result, pattern))
    {
        std::cout << "Match found!\n";
        std::cout << "First word: " << result[1] << "\n";
        std::cout << "Second word: " << result[2] << "\n";
        std::cout << "third word: " << result[3] << "\n";
        std::cout << "forth word: " << result[4] << "\n";
        std::cout << "fifth word: " << result[5] << "\n";
    }
    else
    {
        std::cout << "Match not found.\n";
    }
    return 0;
}