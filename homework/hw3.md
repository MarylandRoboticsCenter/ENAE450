**Due date: EOD Thursday, Mar 7**

**Submission guidelines:**

* create two packages (one per problem): `hw3_1` and `hw3_2`
* both packages should be in the same workspace
* add `README.txt` file into the workspace folder (i.e. `src` and `README.txt` will be in the same folder)
* archive `src` folder and `README.txt`
    * use the following pattern to name the archive:\
     `hw3_<LastName>_<FirstName>`
    * terminal command to create archive:\
    `tar -czf hw3_<LastName>_<FirstName>.tgz src/ README.txt`
* submit the archive

Note:
* For an integer you can use the following topic message: UInt32 from std_msgs, i.e.: `from std_msgs.msg import UInt32`
* For arrays of integers you can use the following topic message: UInt32MultiArray from std_msgs, i.e.: `from std_msgs.msg import UInt32MultiArray`

1. ROS2 Pub & Sub: Sorting  (10 points)
    1. Publisher node: 
        * With the frequency of 0.5Hz generate two random unsorted lists of 10 non-negative integers, with the highest integer value of 100
        * Append to the beginning of each list the serial number of the message starting with 1        
        * With the frequency of 0.5Hz publish the generated lists on topics `topic_1` and `topic_2`
    2. Subscriber node:
        * Subscribe to the topics `topic_1` and `topic_2`
        * For each incoming message **of the same serial number**, merge the lists of numbers and sort the result
        * Print to the console the serial number of the message (on one line) and the merged and sorted list of numbers ((on next line)) 
        * Bonus 5 points: 
    3. Bonus (5 points):
        * Subscriber node publishes topic `topic_3` with the merged and sorted list (append the serial number of the message to the beginning)
    4. Bonus (10 points):
        * Subscriber node spawns a service server that offers three different sorting algorithms. Service request message: name of the sorting algorithm. Service response message: TRUE or FALSE depending on if the algorithm is available. Sorting of the array has to be done with the chosen algorithm.
        * Publisher node sends a client service request (once) to choose the sorting algorithm

2. ROS2 Pub & Sub: Number Search (10 points)
    1. Publisher node: 
        * With the frequency of 0.5Hz generate a random unsorted list of 21 non-negative integers, with the highest integer value of 100
        * With the frequency of 0.5Hz generate a random integer within the range of 1 and 100
        * Append to the beginning of the list and the integer the serial number of the message starting with 1                
        * With the frequency of 0.5Hz publish the generated list and the integer on topics `topic_1` and `topic_2`
    2. Subscriber node:
        * Subscribe to the topics `topic_1` and `topic_2`
        * For each incoming message **of the same serial number**, search for the integer from `topic_2` in the array from `topic_1`
        * Print to the console the following: serial number of the message (one line), the integer from `topic_2` (next line), TRUE or FALSE depending on if the search was successful or not (next line).
    3. Bonus (10 points):
        * Subscriber node spawns a service server that is expecting the integer to search for in the array. Service request message: serial number of the message and the integer to search for. Service response message: TRUE or FALSE depending on if the search was successful or not. 
        * Publisher node publishes only one topic (`topic_1`). Every time the publisher sends topic message it send the serice request with the integer to search for. If the response is TRUE, it prints SUCCESS to the console.

