import time

def aguardar_motores(motores, posicoes_destino, tolerancia=20, timeout=5000.0):
    """
    Espera os motores chegarem próximos da posição desejada.
    """
    inicio = time.time()
    while True:
        todos_alcancaram = True
        for i, pos_desejada in enumerate(posicoes_destino, start=1):
            if i not in motores:
                continue
            pos_atual = motores[i].get_present_position()
            if abs(pos_atual - pos_desejada) > tolerancia:
                todos_alcancaram = False
                break

        if todos_alcancaram:
            return True

        if (time.time() - inicio) > timeout:
            print("⚠️ Timeout aguardando motores.")
            return False

        time.sleep(0.05)


def executar_sequencia(lista_comandos, motores, manipulador):
    """
    Executa uma lista de posições. Cada item da lista deve ser uma lista com 5 posições [m1, m2, m3, m4, m5]
    """
    from excute_manip import executar_movimento_seguindo_dh

    for idx, comando in enumerate(lista_comandos):
        print(f"\n📦 Executando comando {idx+1}: {comando}")
        sucesso = executar_movimento_seguindo_dh(comando, motores)

        if not sucesso:
            print("⛔ Movimento bloqueado por risco de colisão. Pulando para o próximo.")
            continue

        ok = aguardar_motores(motores, comando)
        if ok:
            print("✅ Comando concluído.")
        else:
            print("⚠️ Motores não chegaram a tempo. Continuando...")

        time.sleep(0.5)  # pequeno intervalo entre comandos
