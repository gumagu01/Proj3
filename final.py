import numpy as np
import matplotlib.pyplot as plt

# Encontra a direção do vetor 
def normalize(vector):
    return vector / np.linalg.norm(vector)


def ponto_de_intersecao(center, raio, ray_origin, ray_direction):
    b = 2 * np.dot(ray_direction, ray_origin - center)
    c = np.linalg.norm(ray_origin - center) ** 2 - raio ** 2
    delta = b ** 2 - 4 * c
    if delta > 0:
        r1 = (-b + np.sqrt(delta)) / 2
        r2 = (-b - np.sqrt(delta)) / 2
        if r1 > 0 and r2 > 0:
            if r1 <= r2:
                return r1
            else:
                return r2
    return None

# verifica se ha objeto ou é apenas vazio
def ha_objeto(esferas, ray_origin, ray_direction):
    distance = ponto_de_intersecao(esferas['center'], esferas['raio'], ray_origin, ray_direction)
    esfera = None
    min_distance = np.inf
    # verifica se tem uma esfera na direção do raio 
    if distance and distance < min_distance:
            min_distance = distance
            esfera = esferas
    return esfera, min_distance


# ------------ BLOCO DE DEFINIÇÃO DOS PARÂMETROS DO PROBLEMA -----------------
width = 300
height = 300

camera = np.array([0, 0, 1])

ratio = float(width) / height

tela = (-1, 1 / float(width) / height, 1, -1 / float(width) / height) # left, top, right, bottom


esferas = { 'center': np.array([-0.2, 0, -1]), 'raio': 1, 'ambient': np.array([0.1, 0, 0]), 'diffuse': np.array([0.7, 0, 0]), 'specular': np.array([1, 1, 1]), 'n': 25 }


luz = { 'position': np.array([5, 5, 5]), 'ambient': np.array([1, 1, 1]), 'diffuse': np.array([1, 1, 1]), 'specular': np.array([1, 1, 1]) }

image = np.zeros((height, width, 3))
# ------------ FIM DO BLOCO --------------------------------------------------


# Função principal que preenche a imagem segundo aspectos da cor 
for i, y in enumerate(np.linspace(tela[1], tela[3], height)):
    for j, x in enumerate(np.linspace(tela[0], tela[2], width)):
        pixel = np.array([x, y, 0])
        origin = camera
        direction = normalize(pixel - origin)

         # check for intersections
        esfera, min_distance = ha_objeto(esferas, origin, direction)
        if esfera is None:
            continue

        # compute intersection point between ray and nearest object
        intersection = origin + min_distance * direction

        normal_to_surface = normalize(intersection - esfera['center'])
        shifted_point = intersection + 1e-5 * normal_to_surface
        intersection_to_luz = normalize(luz['position'] - shifted_point)


        # Criando vetor RGB
        illumination = np.zeros((3))

        # Iluminação ambiente 
        illumination += esfera['ambient'] * luz['ambient']

        # Componente difusa
        illumination += esfera['diffuse'] * luz['diffuse'] * np.dot(intersection_to_luz, normal_to_surface)

        # Componente especular
        intersection_to_camera = normalize(camera - intersection)
        H = normalize(intersection_to_luz + intersection_to_camera)
        illumination += esfera['specular'] * luz['specular'] * np.dot(normal_to_surface, H) ** (esfera['n'])

        image[i, j] = np.clip(illumination, 0, 1)


plt.imsave('image3.png', image)