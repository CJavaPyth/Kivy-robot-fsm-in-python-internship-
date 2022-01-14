import yaml

with open('goals.yaml') as file:
    goals_list = yaml.load(file, Loader=yaml.FullLoader)

print(goals_list)
