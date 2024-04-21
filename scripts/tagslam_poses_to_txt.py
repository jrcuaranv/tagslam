"""
Converting initial poses obtimized with tagslam https://github.com/berndpfrommer/tagslam_root
to .csv file
"""
import yaml

# Specify the path to your YAML file
yaml_file_path = "/home/daslab/tagslam_root/src/tagslam/example_zed2i_apt5/poses.yaml"

#output file
apriltags_file = open('/home/daslab/Documents/repositories/pose_estimation/src/tagslam/data/apriltags_apt5.csv', 'w+')



# Open the YAML file and load its contents into a Python dictionary
with open(yaml_file_path, 'r') as yaml_file:
    yaml_data = yaml.safe_load(yaml_file)

# Now you can work with the 'yaml_data' dictionary
for tags in yaml_data['bodies'][0]['grasp_lab']['tags']:
    tag_id = tags['id']
    x = tags['pose']['position']['x']
    y =  tags['pose']['position']['y']
    z =  tags['pose']['position']['z']
    rx =  tags['pose']['rotation']['x']
    ry =  tags['pose']['rotation']['y']
    rz =  tags['pose']['rotation']['z']
    apriltags_file.write(str(tag_id) + ',' + str(x)+ ',' + str(y)+ ',' + str(z)+ ',' + str(rx)+ ',' + str(ry)+ ',' + str(rz)+ '\n')


# print(yaml_data['bodies'][0]['grasp_lab']['tags'][0])

apriltags_file.close()