import yaml
import os
from ament_index_python.packages import get_package_share_directory



def extract_map_configuration(world,in_simulation): 
    # Caminho do YAML
    config_path = os.path.join(
        get_package_share_directory('omnicare_behavior'),
        'config',
        'floor_maps_params.yaml'
    )

    # Carregar o YAML
    with open(config_path, 'r') as file:
        maps_config = yaml.safe_load(file)
    
    # Verificar qual mundo está sendo selecionado
    world_map = maps_config["FEI"] if world == "FEI" else maps_config["HU_USP"]

    # Adiciona o prefixo do pacote nos caminhos
    for floor_dict in [world_map["real"], world_map["sim"]]:
        for floor, data in floor_dict.items():
            data["map_path"] = os.path.join(get_package_share_directory('navigation_pkg'),data["map_path"])
            
    # Verificação se a variavel passada não for booleana, seta como False 
    if in_simulation not in [True, False]:
        in_simulation = False


    # Acesso aos dicionários
    floor_map = world_map["sim"] if in_simulation else world_map["real"]

    return floor_map
