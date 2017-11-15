import random
import numpy

ACTIONS = ["up", "down", "right", "left"]

map =  [[-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
        [-1, 0, 0, 0, 0, 0, 0,-1,-1, 0, 0,-1],
        [-1,-1, 0,-1, 0, 0,-1,-1,-1, 0, 0,-1],
        [-1, 0,-1,-1,-1, 0,-1, 0, 0, 0, 0,-1],
        [-1,-1, 0, 0,-1, 0, 0, 0, 0, 0, 0,-1],
        [-1, 0, 0, 0,-1,-1, 0, 0,-1, 0,-1,-1],
        [-1,-1,-1,10, 0, 0, 0,-1,-1, 0, 0,-1],
        [-1, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0,-1],
        [-1, 0, 0,-1, 0, 0, 0,-1, 0, 0, 0,-1],
        [-1, 0, 0,-1, 0, 0, 0, 0,-1,-1,-1,-1],
        [-1, 0,-1, 0,-1, 0, 0,-1, 0,-1,-1,-1],
        [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]]


gamma = 0
initial_state = (1,2)
goal_state = (6,3)
state = initial_state
action = None

q_table = [[0]*4 for i in range(100)]

i = state[0]
j = state[1]

reward_table =  [[ map[i-1][j-1], map[i-1][j], map[i-1][j+1]], 
                [ map[i][j-1],   map[i][j],   map[i][j+1]   ], 
                [ map[i+1][j-1], map[i+1][j], map[i+1][j+1] ]]


counter = 0
for x in reward_table:
    for y in x:

        if y == 10:
            if counter == 1:
                q_table[(i)*12+(j+1)][0] = y
                possible_actions += "up"
            if counter == 7:
                q_table[(i)*12+(j+1)][1] = y
                possible_actions += "down"
            if  counter == 5:
                q_table[(i)*12+(j+1)][2] = y
                possible_actions += "right"
            if  counter == 3:
                q_table[(i)*12+(j+1)][3] = y
                possible_actions += "left"

        elif y == 1:
            if counter == 1:
                q_table[(i)*12+(j+1)][0] = y
                if "up" not in possible_actions: possible_actions += "up"
            if counter == 7:
                q_table[(i)*12+(j+1)][1] = y
                if "down" not in possible_actions: possible_actions += "down"
            if  counter == 5:
                q_table[(i)*12+(j+1)][2] = y
                if "right" not in possible_actions: possible_actions += "right"
            if  counter == 3:
                q_table[(i)*12+(j+1)][3] = y
                if "left" not in possible_actions: possible_actions += "left"


        counter += 1

            

#while state != goal_state:
    
    

