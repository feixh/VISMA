shaders = [
    ('position_texture', 'vert'),
    ('position_color_texture', 'vert'),
    ('basic_mvp', 'vert'),
    ('edge_detection', 'frag'),
    ('depth_linearization', 'frag')
]


for shader_name, shader_type in shaders:
    s = '#pragma once\nnamespace feh {{\n' \
        '#include <string>\nstatic const std::string {}_{} = R"(\n'.format(shader_name, shader_type)
    with open('shaders/'+shader_name+'.'+shader_type, 'r') as f:
        for line in f:
            s += line

    s += '\n)";\n}'

    with open('shaders/'+shader_name+'_'+shader_type+'.i', 'w') as f:
        f.write(s)



