#include <iostream>

int fibonacci(int n) {
    if (n <= 1) {
        return n;
    }
    return fibonacci(n - 1) + fibonacci(n - 2);
}

int main() {
    int n;
    std::cout << "Enter a number: ";
    std::cin >> n;
    if (n >= 0) {
        std::cout << n << "th Fibonacci number is " << fibonacci(n) << std::endl;
    } else {
        std::cout << "Invalid input" << std::endl;
    }
    return 0;
}
