import csv
import matplotlib.pyplot as plt
import math
import numpy as np # Adicione esta linha

def ler_csv(caminho_arquivo, withoutGoal = False):
    tempo = []
    goal_x = []
    goal_y = []
    robot_x = []
    robot_y = []
    angular_error = []
    linear_error = []
    with open(caminho_arquivo, newline='') as csvfile:
        leitor = csv.DictReader(csvfile)
        for linha in leitor:
            tempo.append(float(linha['timestamp']))
            robot_x.append(float(linha['robot_x']))
            robot_y.append(float(linha['robot_y']))
            if(not(withoutGoal)):
                goal_x.append(float(linha['goal_x']))
                goal_y.append(float(linha['goal_y']))
            angular_error.append(float(linha['angular_error']))
            linear_error.append(float(linha['linear_error']))

    if(withoutGoal):
        return tempo,robot_x,robot_y,angular_error,linear_error

    return tempo,goal_x,goal_y,robot_x,robot_y,angular_error,linear_error

def calcular_estatisticas(goal_x, goal_y, robot_x, robot_y):
    # 1. Erro Linear (Distância Euclidiana Ponto a Ponto)
    erros_lineares_calc = [np.sqrt((gx - rx)**2 + (gy - ry)**2) for gx, gy, rx, ry in zip(goal_x, goal_y, robot_x, robot_y)]
    
    media_linear = np.mean(erros_lineares_calc)
    desvio_linear = np.std(erros_lineares_calc)
    
    # 2. Erro Angular Calculado pelas Posições (Diferença de Orientação)
    erros_angulares_calc = []
    
    # Iteramos a partir do índice 1 para comparar com o ponto anterior (índice i-1)
    for i in range(1, len(goal_x)):
        # Vetor do trajeto alvo
        dg_x = goal_x[i] - goal_x[i-1]
        dg_y = goal_y[i] - goal_y[i-1]
        
        # Vetor do trajeto do robô
        dr_x = robot_x[i] - robot_x[i-1]
        dr_y = robot_y[i] - robot_y[i-1]
        
        # Se o robô ou alvo não se moveram neste timestamp, ignoramos para evitar divisões/ângulos falsos
        if (dr_x == 0 and dr_y == 0) or (dg_x == 0 and dg_y == 0):
            continue
            
        # Orientação via arco-tangente (retorna em radianos)
        theta_goal = np.arctan2(dg_y, dg_x)
        theta_robot = np.arctan2(dr_y, dr_x)
        
        # Diferença angular
        diff = theta_goal - theta_robot
        
        # Normalizar o ângulo para ficar no menor caminho entre -pi e pi
        diff_norm = np.arctan2(np.sin(diff), np.cos(diff))
        
        # Converter para graus e extrair o valor absoluto (para não zerar a média com os sinais opostos)
        erros_angulares_calc.append(np.degrees(abs(diff_norm)))
        
    if erros_angulares_calc:
        media_angular = np.mean(erros_angulares_calc)
        desvio_angular = np.std(erros_angulares_calc)
    else:
        media_angular, desvio_angular = 0.0, 0.0
        
    return media_linear, desvio_linear, media_angular, desvio_angular

def calcular_estatisticas_iteracao(goal_x, goal_y, robot_x, robot_y, usar_proximidade=True):
    """
    Calcula os erros estatísticos (Média e Variância/Desvio Padrão) da iteração.
    Se usar_proximidade=True, busca o ponto mais próximo no alvo.
    Se usar_proximidade=False, compara ponto a ponto (truncando pelo menor tamanho).
    """
    if not usar_proximidade:
        # ==========================================
        # FORMATO ANTERIOR: Ponto a Ponto (Truncado)
        # ==========================================
        tamanho = min(len(goal_x), len(robot_x))
        gx_trunc, gy_trunc = goal_x[:tamanho], goal_y[:tamanho]
        rx_trunc, ry_trunc = robot_x[:tamanho], robot_y[:tamanho]
        
        # Erro Linear
        erros_lineares = [np.sqrt((gx - rx)**2 + (gy - ry)**2) for gx, gy, rx, ry in zip(gx_trunc, gy_trunc, rx_trunc, ry_trunc)]
        
        # Erro Angular
        erros_angulares = []
        for i in range(1, tamanho):
            dg_x = gx_trunc[i] - gx_trunc[i-1]
            dg_y = gy_trunc[i] - gy_trunc[i-1]
            dr_x = rx_trunc[i] - rx_trunc[i-1]
            dr_y = ry_trunc[i] - ry_trunc[i-1]
            
            if (dr_x == 0 and dr_y == 0) or (dg_x == 0 and dg_y == 0):
                continue
                
            theta_goal = np.arctan2(dg_y, dg_x)
            theta_robot = np.arctan2(dr_y, dr_x)
            diff = theta_goal - theta_robot
            diff_norm = np.arctan2(np.sin(diff), np.cos(diff)) 
            erros_angulares.append(np.degrees(abs(diff_norm)))
            
    else:
        # ==========================================
        # FORMATO ATUAL: Ponto Mais Próximo
        # ==========================================
        gx_array = np.array(goal_x)
        gy_array = np.array(goal_y)
        
        erros_lineares = []
        erros_angulares = []
        
        for i in range(1, len(robot_x)):
            rx, ry = robot_x[i], robot_y[i]
            rx_prev, ry_prev = robot_x[i-1], robot_y[i-1]
            
            dr_x = rx - rx_prev
            dr_y = ry - ry_prev
            if dr_x == 0 and dr_y == 0:
                continue
                
            # Busca do ponto mais próximo
            distancias = np.sqrt((gx_array - rx)**2 + (gy_array - ry)**2)
            idx_mais_proximo = np.argmin(distancias)
            
            erros_lineares.append(distancias[idx_mais_proximo])
            
            idx_g = idx_mais_proximo
            if idx_g == 0:
                idx_g = 1
                
            dg_x = gx_array[idx_g] - gx_array[idx_g-1]
            dg_y = gy_array[idx_g] - gy_array[idx_g-1]
            
            if dg_x == 0 and dg_y == 0:
                continue
                
            theta_goal = np.arctan2(dg_y, dg_x)
            theta_robot = np.arctan2(dr_y, dr_x)
            diff = theta_goal - theta_robot
            diff_norm = np.arctan2(np.sin(diff), np.cos(diff)) 
            erros_angulares.append(np.degrees(abs(diff_norm)))

    # Cálculos Estatísticos Finais Comuns a ambos os métodos
    std_linear = np.std(erros_lineares) if erros_lineares else 0.0
    var_linear = np.var(erros_lineares) if erros_lineares else 0.0
    
    if erros_angulares:
        media_angular = np.mean(erros_angulares)
        var_angular = np.var(erros_angulares)
    else:
        media_angular, var_angular = 0.0, 0.0
        
    return std_linear, var_linear, media_angular, var_angular

def obter_erros_historico(goal_x, goal_y, robot_x, robot_y, tempo, usar_proximidade=True):
    """
    Retorna arrays contendo a evolução dos erros ao longo do tempo.
    Se usar_proximidade=True, busca o ponto mais próximo no alvo.
    Se usar_proximidade=False, compara ponto a ponto (truncando pelo menor tamanho).
    """
    if not usar_proximidade:
        # ==========================================
        # FORMATO ANTERIOR: Ponto a Ponto (Truncado)
        # ==========================================
        tamanho = min(len(goal_x), len(robot_x))
        gx, gy = goal_x[:tamanho], goal_y[:tamanho]
        rx, ry = robot_x[:tamanho], robot_y[:tamanho]
        t = tempo[:tamanho]

        # Erro Linear
        erros_lineares = [np.sqrt((gx[i] - rx[i])**2 + (gy[i] - ry[i])**2) for i in range(tamanho)]
        tempos_lineares = t

        # Erro Angular
        erros_angulares = []
        tempos_angulares = []

        for i in range(1, tamanho):
            dg_x = gx[i] - gx[i-1]
            dg_y = gy[i] - gy[i-1]
            dr_x = rx[i] - rx[i-1]
            dr_y = ry[i] - ry[i-1]

            if (dr_x == 0 and dr_y == 0) or (dg_x == 0 and dg_y == 0):
                erros_angulares.append(erros_angulares[-1] if erros_angulares else 0.0)
                tempos_angulares.append(t[i])
                continue

            theta_goal = np.arctan2(dg_y, dg_x)
            theta_robot = np.arctan2(dr_y, dr_x)
            diff = theta_goal - theta_robot
            diff_norm = np.arctan2(np.sin(diff), np.cos(diff)) 
            erros_angulares.append(np.degrees(abs(diff_norm)))
            tempos_angulares.append(t[i])
            
        return tempos_lineares, erros_lineares, tempos_angulares, erros_angulares

    else:
        # ==========================================
        # FORMATO ATUAL: Ponto Mais Próximo
        # ==========================================
        gx_array = np.array(goal_x)
        gy_array = np.array(goal_y)
        
        erros_lineares = []
        tempos_lineares = []
        erros_angulares = []
        tempos_angulares = []

        # Tratamento inicial (índice 0)
        if len(robot_x) > 0:
            dist_inicial = np.sqrt((gx_array - robot_x[0])**2 + (gy_array - robot_y[0])**2)
            erros_lineares.append(np.min(dist_inicial))
            tempos_lineares.append(tempo[0])

        for i in range(1, len(robot_x)):
            rx, ry = robot_x[i], robot_y[i]
            rx_prev, ry_prev = robot_x[i-1], robot_y[i-1]
            t_atual = tempo[i]
            
            # Ponto mais próximo
            distancias = np.sqrt((gx_array - rx)**2 + (gy_array - ry)**2)
            idx_mais_proximo = np.argmin(distancias)
            
            erros_lineares.append(distancias[idx_mais_proximo])
            tempos_lineares.append(t_atual)
            
            # Erro Angular
            dr_x = rx - rx_prev
            dr_y = ry - ry_prev
            
            if dr_x == 0 and dr_y == 0:
                erros_angulares.append(erros_angulares[-1] if erros_angulares else 0.0)
                tempos_angulares.append(t_atual)
                continue
                
            idx_g = idx_mais_proximo
            if idx_g == 0:
                idx_g = 1 
                
            dg_x = gx_array[idx_g] - gx_array[idx_g-1]
            dg_y = gy_array[idx_g] - gy_array[idx_g-1]
            
            if dg_x == 0 and dg_y == 0:
                erros_angulares.append(erros_angulares[-1] if erros_angulares else 0.0)
                tempos_angulares.append(t_atual)
                continue
                
            theta_goal = np.arctan2(dg_y, dg_x)
            theta_robot = np.arctan2(dr_y, dr_x)
            diff = theta_goal - theta_robot
            diff_norm = np.arctan2(np.sin(diff), np.cos(diff)) 
            erros_angulares.append(np.degrees(abs(diff_norm)))
            tempos_angulares.append(t_atual)
            
        return tempos_lineares, erros_lineares, tempos_angulares, erros_angulares
    
def plotar_erro_tempo(x_dados, y_dados, titulo, ylabel, xlabel='Time (s)', cor='blue'):
    """
    Gera e salva o gráfico da evolução do erro ao longo do tempo em SVG.
    """
    fig = plt.figure(figsize=(12, 5))
    plt.plot(x_dados, y_dados, label='Measured Error', color=cor, linewidth=2)
    plt.xlabel(xlabel, fontsize=16)
    plt.ylabel(ylabel, fontsize=16)
    plt.title(titulo, fontsize=18)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    
    nome_arquivo = titulo.replace(" - ", "_").replace(" ", "_").lower()
    fig.savefig(f"image/{nome_arquivo}.eps", format='eps', bbox_inches='tight')

def plotar_grafico_posicao(resultados_csv, titulo, goal_ref=None, cor_alvo='green', limites_x=None, limites_y=None, indices_falha=None, cor_unica=None, limite_pontos=None):
    """
    Gera gráficos de posição e os salva em formato vetorial (.svg).
    
    :param resultados_csv: Lista contendo os retornos diretos da função ler_csv().
    :param titulo: Título do gráfico e nome base do arquivo gerado.
    :param goal_ref: Tupla (goal_x, goal_y) para trajeto alvo (necessário se withoutGoal=True).
    :param cor_alvo: Cor da linha do percurso alvo.
    :param limites_x, limites_y: Tuplas (min, max) para focar em áreas específicas (zoom).
    :param indices_falha: Tupla (inicio, fim) para destacar o trecho de falha em amarelo.
    :param cor_unica: Cor aplicada a todas as iterações (útil para gráficos de iteração única).
    :param limite_pontos: Quantidade de pontos a exibir (útil para truncar trajetórias falhas).
    """
    fig = plt.figure(figsize=(10, 10))
    
    # 1. Definir o percurso alvo
    if goal_ref:
        goal_x, goal_y = goal_ref
    else:
        # Se não houver ref externa, extrai do primeiro resultado (Virtual)
        goal_x, goal_y = resultados_csv[0][1], resultados_csv[0][2]
        
    plt.plot(goal_x, goal_y, label='Target Path', color=cor_alvo, linewidth=5)
    
    # 2. Plotar o trecho de falha (se configurado)
    if indices_falha is not None:
        inicio, fim = indices_falha
        plt.plot(goal_x[inicio:fim], goal_y[inicio:fim], label='Line Failure', color='yellow', linewidth=10)
        
    # 3. Processar e plotar as trajetórias do robô internamente
    for i, resultado in enumerate(resultados_csv):
        # Desempacotamento dinâmico: Virtual tem 7 valores, Visual tem 5
        if len(resultado) == 7:
            _, _, _, rx, ry, _, _ = resultado
        else:
            _, rx, ry, _, _ = resultado
            
        # Truncar os pontos se o parâmetro limite_pontos for passado
        if limite_pontos:
            rx = rx[:limite_pontos]
            ry = ry[:limite_pontos]
            
        label = f'Iteration {i + 1}'
        
        # Plota os scatters com cor fixa ou gerada automaticamente pelo Matplotlib
        if cor_unica:
            plt.scatter(rx, ry, label=label, color=cor_unica)
        else:
            plt.scatter(rx, ry, label=label)
            
    # 4. Configurações de Eixos e Layout
    plt.xlabel('X Position (m)', fontsize=18)
    plt.ylabel('Y Position (m)', fontsize=18)
    plt.title(titulo, fontsize=20)
    plt.grid(True)
    plt.legend()
    
    if limites_x is not None: plt.xlim(limites_x)
    if limites_y is not None: plt.ylim(limites_y)
        
    # 5. Salvar em formato Vetorial (SVG/EPS)
    nome_arquivo = titulo.replace(" - ", "_").replace(" ", "_").lower()
    
    # bbox_inches='tight' garante que o texto e legendas não fiquem cortados na exportação vetorial
    fig.savefig(f"image/{nome_arquivo}.eps", format='eps', bbox_inches='tight')
    

if __name__ == "__main__":
    line_fail_csv = ["src/virtual_line_follower/scripts/logs/line_fail_logs/1791033039.7144978_log.csv",
                    "src/robot_line_follower/scripts/logs/line_fail_logs/1791820981_log.csv"]

    maze_csv = ["src/virtual_line_follower/scripts/logs/maze_logs/1790984078.649412_log.csv",
                "src/robot_line_follower/scripts/logs/maze_logs/1790869117_log.csv"]
    
    repeated_virtual_csv = ["src/virtual_line_follower/scripts/logs/square_logs/1790294617.3354707_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790295776.713717_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790296913.6762493_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790297978.4115157_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790299003.0440211_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790300148.3903897_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790344721.3329697_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790343513.2763941_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790342198.4888725_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790340942.7862062_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790339680.634838_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790338623.3430147_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790337476.3753707_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790336328.893584_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790335302.4101493_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790334182.7199235_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790306108.2144375_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790304252.5453212_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790302373.695688_log.csv",
                            "src/virtual_line_follower/scripts/logs/square_logs/1790301300.323143_log.csv"]
    
    repeated_real_csv = ["src/robot_line_follower/scripts/logs/square_logs/1790722567_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790723438_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790724332_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790725228_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790858352_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790726884_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790727431_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790727964_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790728509_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790729039_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790729585_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790730128_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790730656_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790731246_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790731822_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790732388_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790732948_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790733515_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790734640_log.csv",
                        "src/robot_line_follower/scripts/logs/square_logs/1790734080_log.csv"]

    ####################################### SQUARE LOGS ###############################################

    # Lê todos os arquivos virtuais de uma vez
    resultados_virtuais = [ler_csv(arq) for arq in repeated_virtual_csv]

    # Figura 1: Trajeto Completo Virtual
    plotar_grafico_posicao(
        resultados_csv=resultados_virtuais,
        titulo="Executed Trajectory - Virtual Line Follower - Repeatability"
    )

    # Figura 2: Foco na Curva Virtual
    plotar_grafico_posicao(
        resultados_csv=resultados_virtuais,
        titulo="Executed Trajectory - Virtual Line Follower - Focus on Curve",
        cor_alvo='red',
        limites_x=(15.2, 16.2),
        limites_y=(15.2, 16.2)
    )

    # Lê todos os arquivos visuais (withoutGoal=True)
    resultados_visuais = [ler_csv(arq, withoutGoal=True) for arq in repeated_real_csv]
    
    # Extrai as coordenadas alvo de um resultado virtual para servir de referência
    ref_alvo = (resultados_virtuais[0][1], resultados_virtuais[0][2])

    # Figura 3: Trajeto Completo Visual
    plotar_grafico_posicao(
        resultados_csv=resultados_visuais,
        titulo="Executed Trajectory - Visual Line Follower - Repeatability",
        goal_ref=ref_alvo
    )

    # Figura 4: Foco na Curva Visual
    plotar_grafico_posicao(
        resultados_csv=resultados_visuais,
        titulo="Executed Trajectory - Visual Line Follower - Focus on Curve",
        goal_ref=ref_alvo,
        cor_alvo='red',
        limites_x=(15.2, 16.2),
        limites_y=(15.2, 16.2)
    )

    ######################################## MAZE LOGS ##############################################

    # Lê todos os arquivos virtuais de uma vez
    resultados_virtuais = [ler_csv(arq) for arq in [maze_csv[0]]]

    # Figura 5: Trajeto Completo Virtual
    plotar_grafico_posicao(
        resultados_csv=resultados_virtuais,
        titulo="Executed Trajectory - Virtual Line Follower - Complexity"
    )

    # Lê todos os arquivos visuais (withoutGoal=True)
    resultados_visuais = [ler_csv(arq, withoutGoal=True) for arq in [maze_csv[1]]]
    
    # Extrai as coordenadas alvo de um resultado virtual para servir de referência
    ref_alvo = (resultados_virtuais[0][1], resultados_virtuais[0][2])

    # Figura 6: Trajeto Completo Visual
    plotar_grafico_posicao(
        resultados_csv=resultados_visuais,
        titulo="Executed Trajectory - Visual Line Follower - Complexity",
        goal_ref=ref_alvo
    )

    ######################################## Line Fail LOGS ##############################################

    # Lê todos os arquivos virtuais de uma vez
    resultados_virtuais = [ler_csv(arq) for arq in [line_fail_csv[0]]]

    # Figura 5: Trajeto Completo Virtual
    plotar_grafico_posicao(
        resultados_csv=resultados_virtuais,
        titulo="Executed Trajectory - Virtual Line Follower - Line Failure",
        indices_falha=[21000, 22000]
    )

    # Lê todos os arquivos visuais (withoutGoal=True)
    resultados_visuais = [ler_csv(arq, withoutGoal=True) for arq in [line_fail_csv[1]]]
    
    # Extrai as coordenadas alvo de um resultado virtual para servir de referência
    ref_alvo = (resultados_virtuais[0][1], resultados_virtuais[0][2])

    # Figura 6: Trajeto Completo Visual
    plotar_grafico_posicao(
        resultados_csv=resultados_visuais,
        titulo="Executed Trajectory - Visual Line Follower - Line Failure",
        goal_ref=ref_alvo,
        indices_falha=[21000, 22000]
    )

    # =====================================================================
    # GERAÇÃO DA TABELA DE COMPARAÇÃO EM IMAGEM (VIRTUAL VS VISUAL)
    # =====================================================================

    # Obter a trajetória alvo de referência (do primeiro ficheiro virtual)
    _, goal_x_ref, goal_y_ref, _, _, _, _ = ler_csv(repeated_virtual_csv[0], withoutGoal=False)

    # --- Processamento: Seguidor Virtual ---
    tempos_virt, stds_lin_virt, vars_lin_virt, meds_ang_virt, vars_ang_virt = [], [], [], [], []
    for arquivo in repeated_virtual_csv:
        tempo, gx, gy, rx, ry, _, _ = ler_csv(arquivo, withoutGoal=False)
        tempos_virt.append(tempo[-1] - tempo[0])
        std_lin, var_lin, med_ang, var_ang = calcular_estatisticas_iteracao(gx, gy, rx, ry)
        stds_lin_virt.append(std_lin); vars_lin_virt.append(var_lin)
        meds_ang_virt.append(med_ang); vars_ang_virt.append(var_ang)

    # --- Processamento: Seguidor Visual ---
    tempos_vis, stds_lin_vis, vars_lin_vis, meds_ang_vis, vars_ang_vis = [], [], [], [], []
    for arquivo in repeated_real_csv:
        tempo, rx, ry, _, _ = ler_csv(arquivo, withoutGoal=True)
        tempos_vis.append(tempo[-1] - tempo[0])
        std_lin, var_lin, med_ang, var_ang = calcular_estatisticas_iteracao(goal_x_ref, goal_y_ref, rx, ry)
        stds_lin_vis.append(std_lin); vars_lin_vis.append(var_lin)
        meds_ang_vis.append(med_ang); vars_ang_vis.append(var_ang)

    # --- Desenhar a Tabela Única com Matplotlib ---
    fig17, ax = plt.subplots(figsize=(14, 3))
    ax.axis('tight')
    ax.axis('off')

    # Definir colunas e formatar os dados calculados para a tabela
    colunas = [
        "Method", 
        "Average Route Time (s)", 
        "Average Linear\nStandard Deviation (m)", 
        "Variance of Linear\nDeviation (m²)", 
        "Average Angular\nDeviation (°)", 
        "Variance of Angular\nDeviation (°²)"
    ]

    valores = [
        [
            "Virtual Follower", 
            f"{np.mean(tempos_virt):.2f}", 
            f"{np.mean(stds_lin_virt):.4f}", 
            f"{np.mean(vars_lin_virt):.6f}", 
            f"{np.mean(meds_ang_virt):.2f}", 
            f"{np.mean(vars_ang_virt):.4f}"
        ],
        [
            "Visual Follower", 
            f"{np.mean(tempos_vis):.2f}", 
            f"{np.mean(stds_lin_vis):.4f}", 
            f"{np.mean(vars_lin_vis):.6f}", 
            f"{np.mean(meds_ang_vis):.2f}", 
            f"{np.mean(vars_ang_vis):.4f}"
        ]
    ]

    # Criar e estilizar a tabela
    tabela = ax.table(cellText=valores, colLabels=colunas, cellLoc='center', loc='center')
    tabela.auto_set_font_size(False)
    tabela.set_fontsize(12)
    tabela.scale(2.0, 2.5) # Ajusta a largura e altura das células

    # Pintar o cabeçalho para destaque
    for (row, col), cell in tabela.get_celld().items():
        if row == 0:
            cell.set_text_props(weight='bold', color='white')
            cell.set_facecolor('#4c72b0')
        elif col == 0:
            cell.set_text_props(weight='bold')
            cell.set_facecolor('#f0f0f0')

    plt.title("Performance Comparison: Virtual Line vs Visual Line", fontsize=16, fontweight='bold', pad=20)

    # Salvar a tabela na mesma pasta das outras imagens
    fig17.savefig("image/tabela_comparativa_desempenho.eps", format='eps', bbox_inches='tight', dpi=300)


    # =====================================================================
    # GRÁFICOS DE ERRO AO LONGO DO TEMPO (Foco na 1ª Iteração de Cada)
    # =====================================================================

    # 1. SEGUIDOR VIRTUAL (Iteração 1)
    # Resultados virtuais tem 7 saídas
    t_virt, gx_virt, gy_virt, rx_virt, ry_virt, _, _ = resultados_virtuais[0]
    
    # Normaliza o tempo para que inicie em t=0
    t_virt_norm = [t - t_virt[0] for t in t_virt]
    
    # Extrai o histórico
    t_lin_v, err_lin_v, t_ang_v, err_ang_v = obter_erros_historico(gx_virt, gy_virt, rx_virt, ry_virt, t_virt_norm)
    
    plotar_erro_tempo(t_lin_v, err_lin_v, "Linear Error over Time - Virtual Follower", "Linear Error (m)", cor='royalblue')
    plotar_erro_tempo(t_ang_v, err_ang_v, "Angular Error over Time - Virtual Follower", "Angular Error (Degrees)", cor='royalblue')

    # 2. SEGUIDOR VISUAL (Iteração 1)
    # Resultados visuais tem 5 saídas
    t_vis, rx_vis, ry_vis, _, _ = resultados_visuais[0]
    
    t_vis_norm = [t - t_vis[0] for t in t_vis]
    
    # Usamos o goal de referência para o visual
    t_lin_r, err_lin_r, t_ang_r, err_ang_r = obter_erros_historico(goal_x_ref, goal_y_ref, rx_vis, ry_vis, t_vis_norm)
    
    plotar_erro_tempo(t_lin_r, err_lin_r, "Linear Error over Time - Visual Follower", "Linear Error (m)", cor='darkorange')
    plotar_erro_tempo(t_ang_r, err_ang_r, "Angular Error over Time - Visual Follower", "Angular Error (Degrees)", cor='darkorange')
