#include <iostream>
#include <istream>
#include <fstream>
#include <stdio.h>
#include <vector>
using namespace std;

class regression {
  protected:
    // Dynamic array which is going
    // to contain all (i-th x)
    vector<double> x;
 
    // Dynamic array which is going
    // to contain all (i-th y)
    vector<double> y;
 
    // Store the coefficient/slope in
    // the best fitting line
    double coeff;
 
    // Store the constant term in
    // the best fitting line
    double constTerm;
 
    // Contains sum of product of
    // all (i-th x) and (i-th y)
    double sum_xy;
 
    // Contains sum of all (i-th x)
    double sum_x;
 
    // Contains sum of all (i-th y)
    double sum_y;
 
    // Contains sum of square of
    // all (i-th x)
    double sum_x_square;
 
    // Contains sum of square of
    // all (i-th y)
    double sum_y_square;
 
public:
    // Constructor to provide the default
    // values to all the terms in the
    // object of class regression
    regression()
    {
        coeff = 0;
        constTerm = 0;
        sum_y = 0;
        sum_y_square = 0;
        sum_x_square = 0;
        sum_x = 0;
        sum_xy = 0;
    }
 
    // Function that calculate the coefficient/
    // slope of the best fitting line
    void calculateCoefficient()
    {
        double N = x.size();
        double numerator
            = (N * sum_xy - sum_x * sum_y);
        double denominator
            = (N * sum_x_square - sum_x * sum_x);
        coeff = numerator / denominator;
    }
 
    // Member function that will calculate
    // the constant term of the best
    // fitting line
    void calculateConstantTerm()
    {
        double N = x.size();
        double numerator
            = (sum_y * sum_x_square - sum_x * sum_xy);
        double denominator
            = (N * sum_x_square - sum_x * sum_x);
        constTerm = numerator / denominator;
    }
 
    // Function that return the number
    // of entries (xi, yi) in the data set
    int sizeOfData()
    {
        return x.size();
    }
 
    // Function that return the coefficient/
    // slope of the best fitting line
    double coefficient()
    {
        if (coeff == 0)
            calculateCoefficient();
        return coeff;
    }
 
    // Function that return the constant
    // term of the best fitting line
    double constant()
    {
        if (constTerm == 0)
            calculateConstantTerm();
        return constTerm;
    }
 
    // Function that print the best
    // fitting line
    void PrintBestFittingLine()
    {
        if (coeff == 0 && constTerm == 0) {
            calculateCoefficient();
            calculateConstantTerm();
        }
        cout << "The best fitting line is y = "
             << coeff << "x + " << constTerm << endl;
    }
 
    // Function to take input from the dataset
    void takeInput(int n)
    {
        for (int i = 0; i < n; i++) {
            // In a csv file all the values of
            // xi and yi are separated by commas
            char comma;
            double xi;
            double yi;
            cin >> xi >> comma >> yi;
            sum_xy += xi * yi;
            sum_x += xi;
            sum_y += yi;
            sum_x_square += xi * xi;
            sum_y_square += yi * yi;
            x.push_back(xi);
            y.push_back(yi);
        }
    }
 
 
    // Function to show the data set
    void showData()
    {
        for (int i = 0; i < 62; i++) {
            printf("_");
        }
        printf("\n\n");
        printf("|%15s%5s %15s%5s%20s\n",
               "X", "", "Y", "", "|");
 
        for (int i = 0; i < x.size(); i++) {
            printf("|%20f %20f%20s\n",
                   x[i], y[i], "|");
        }
 
        for (int i = 0; i < 62; i++) {
            printf("_");
        }
        printf("\n");
    }
 
    // Function to predict the value
    // corresponding to some input
    double predict(double x)
    {
        return coeff * x + constTerm;
    }
 
    // Function that returns overall
    // sum of square of errors
    double errorSquare()
    {
        double ans = 0;
        for (int i = 0;
             i < x.size(); i++) {
            ans += ((predict(x[i]) - y[i])
                    * (predict(x[i]) - y[i]));
        }
        return ans;
    }
 
    // Functions that return the error
    // i.e the difference between the
    // actual value and value predicted
    // by our model
    double errorIn(double num)
    {
        for (int i = 0;
             i < x.size(); i++) {
            if (num == x[i]) {
                return (y[i] - predict(x[i]));
            }
        }
        return 0;
    }
};

