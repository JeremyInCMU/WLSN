# Run with minicom -C <filename>



filename = "assignment3_part2.txt"


# Task 1 variables
task1_count = 0
task1_period_seconds = 1
task1_average_response_time = 0
task1_max_response_time = 0.0

# Task 2 variables
task2_count = 0
task2_period_seconds = 2
task2_average_response_time = 0
task2_max_response_time = 0.0


with open(filename,'r') as f:
    while True:
        line=f.readline()
        if not line: break
        
        # Find the task number
        task_num = line[4]


        # Get the seconds completed
        first_space_index = line.index(" ") + 1
    	line = line[first_space_index:]
    	second_space_index = line.index(" ") + 1
    	seconds = line[:2]
    	try:
    		seconds = int(seconds)
    	except ValueError:
    		continue

    	
    	print seconds

    	# Get the nano seconds completed
    	nanoseconds = line[second_space_index: ]
    	nanoseconds = int(nanoseconds) / 10.0**9

    	print nanoseconds

    	# Find final completion time
    	completion_time = seconds + nanoseconds

    	#print completion_time

        if(task_num == '1'):
        	
        	# Find release time
        	release_time = task1_period_seconds * task1_count

        	response_time = completion_time - release_time

        	print "Task 1 response time = " + str(response_time) + "\n"

        	task1_average_response_time += response_time

        	# Save the max response time
        	if(response_time > task1_max_response_time):
        		task1_max_response_time = response_time

        	task1_count += 1
    	else:

    		# Find release time
    		release_time = task2_period_seconds * task2_count

        	response_time = completion_time - release_time

        	print "Task 2 response time = " + str(response_time) + "\n"

        	task2_average_response_time += response_time

        	# Save the max response time
        	if(response_time > task2_max_response_time):
        		task2_max_response_time = response_time

    		task2_count += 1

# Task 1 information
print "-> Task 1 max response time = " + str(task1_max_response_time)
print "-> Task 1 avg response time = " + str(task1_average_response_time / task1_count) + "\n"

# Task 2 information
print "-> Task 2 max response time = " + str(task2_max_response_time)
print "-> Task 2 avg response time = " + str(task2_average_response_time / task2_count)
