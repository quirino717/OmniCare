import yaml
import os
from ament_index_python.packages import get_package_share_directory



def extract_map_configuration(in_simulation): 
    # Caminho do YAML
    config_path = os.path.join(
        get_package_share_directory('omnicare_behavior'),
        'config',
        'floor_maps_params.yaml'
    )

    # Carregar o YAML
    with open(config_path, 'r') as file:
        maps_config = yaml.safe_load(file)

    # Adiciona o prefixo do pacote nos caminhos
    for floor_dict in [maps_config["real"], maps_config["sim"]]:
        for floor, data in floor_dict.items():
            data["map_path"] = os.path.join(
                get_package_share_directory('navigation_pkg'),
                data["map_path"]
            )
            
    # Acesso aos dicionários
    floor_map = maps_config["sim"] if in_simulation else maps_config["real"]

    return floor_map

