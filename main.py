import subprocess

def main():
    print("🚀 Iniciando sequência: pegada com visão computacional...")
    while True:
        # Executa pegada.py e aguarda finalização
        resultado_pegada = subprocess.run(["python", "pegada.py"])
        
        if resultado_pegada.returncode != 0:
            print("❌ Erro durante a execução da pegada. Encerrando ciclo.")
            
        
            print("✅ Pegada finalizada. Iniciando controle manual...")
                
            
            resultado_discard = subprocess.run(["python", "discard.py"])

            if resultado_discard != 0:
                print("❌ Erro ao executar o controle manual. Encerrando ciclo.")
                break
                
                print("✅ Execução manual concluída. Iniciando nova pegada...")

if __name__ == "__main__":
    main()
