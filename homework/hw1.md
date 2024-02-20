Due date: EOD Thursday, Feb 29

1. Python Only: Merge sorting (5 points)
    1. Generate two random unsorted lists of 50 non-negative integers, with the highest integer value of 1000. In other words, your integers will range from 0 to 1000.
    2. Merge the lists and sort the result
    3. The output should be the following:
        1. Tlwo original lists
        2. Merged and sortyed list

2. Python Only: Binary Search (10 points)
    1. A binary search takes in a sorted list (as an array), then continually compares a search value with the middle of the array. Depending on whether the search value is less than or greater than the middle value, the list is split (divide and conquer) to reduce the search space. The program returns TRUE if the search value is in the list, and FALSE if the search value is not in the list.
    2. Write a program that generates a random unsorted list of 101 non-negative integers, with the highest integer value of 1000. In other words, your integers will range from 0 to 1000.
        1. Sort this list. You may use existing python sorting methods.
        2. Ask the user for an integer value. You can assume the user will provide an integer, but you will need to check that the integer is within the given range of [0,1000]. If the input is outside this range, keep prompting the user for a valid input.
        3. Take your sorted list, the user input search value, and perform a binary search. Your output should be four lines:
            1. The original list
            2. The sorted list
            3. The search value
            4. TRUE or FALSE depending on if the search value is in the list

3. Python only: Splitting into sentences (10 points)
    1. You are given a long string. Separate the string into sentences and append the number of each sentence to its beginning.
    2. The output should be the following:
        1. The original string
        2. List of sentences with appended number