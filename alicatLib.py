#!/usr/bin/env python3
"""
Contrôleur Alicat utilisant les assemblies documentés Real Format
- Assembly 100: Setpoint Request (écriture)
- Assembly 107: Standard Readings (lecture)
pip install pycomm3
"""

from pycomm3 import CIPDriver
import struct
import time
from typing import Optional, Dict, Any

class AlicatController():
    """
    Contrôleur Alicat utilisant les assemblies Real Format documentés
    """
    
    # Assemblies documentés
    SETPOINT_ASSEMBLY = 100    # Pour écriture du setpoint
    READINGS_ASSEMBLY = 107    # Pour lecture des données
    
    def __init__(self, ip_address: str):
        self.ip_address = ip_address
        self.driver = CIPDriver(ip_address)
        self.connected = False
    
    def connect(self) -> bool:
        """Établit la connexion"""
        try:
            self.driver.open()
            self.connected = True
            print(f"✓ Connexion établie avec Alicat à {self.ip_address}")
            return True
        except Exception as e:
            print(f"Erreur de connexion: {e}")
            return False
    
    def disconnect(self):
        """Ferme la connexion"""
        if self.connected:
            try:
                self.driver.close()
                self.connected = False
                print("Connexion fermée")
            except:
                pass
    
    def read_assembly(self, instance: int) -> Optional[bytes]:
        """Lit les données d'un assembly"""
        if not self.connected:
            print("Pas de connexion active")
            return None
        
        try:
            result = self.driver.generic_message(
                service=0x0E,  # Get_Attribute_Single
                class_code=4,  # Assembly Object
                instance=instance,
                attribute=3    # Data attribute
            )
            
            if result and hasattr(result, 'value'):
                return result.value
            return None
            
        except Exception as e:
            print(f"Erreur lecture assembly {instance}: {e}")
            return None
    
    def write_assembly(self, instance: int, data: bytes) -> bool:
        """Écrit des données dans un assembly"""
        if not self.connected:
            print("Pas de connexion active")
            return False
        
        try:
            result = self.driver.generic_message(
                service=0x10,  # Set_Attribute_Single
                class_code=4,  # Assembly Object
                instance=instance,
                attribute=3,   # Data attribute
                request_data=data
            )
            
            return result is not None
            
        except Exception as e:
            print(f"Erreur écriture assembly {instance}: {e}")
            return False
    
    def set_setpoint(self, setpoint: float) -> bool:
        """
        Définit le setpoint via Assembly 100
        
        Args:
            setpoint: Valeur de consigne (32-bit IEEE float)
            
        Returns:
            True si succès
        """
        try:
            # Encode comme float 32-bit IEEE (little-endian)
            data = struct.pack('<f', setpoint)
            success = self.write_assembly(self.SETPOINT_ASSEMBLY, data)
            
            # if success:
            #     print(f"✓ Setpoint défini à {setpoint}")
            # else:
            #     print(f"✗ Échec définition setpoint {setpoint}")
            
            return success
            
        except Exception as e:
            print(f"Erreur setpoint: {e}")
            return False
    
    def get_readings(self) -> Optional[Dict[str, Any]]:
        """
        Lit toutes les données via Assembly 107
        
        Returns:
            Dictionnaire avec toutes les lectures ou None
        """
        data = self.read_assembly(self.READINGS_ASSEMBLY)
        
        if not data or len(data) < 52:  # Minimum 52 bytes pour tous les champs
            print(f"Données insuffisantes: {len(data) if data else 0} bytes (minimum 52 requis)")
            return None
        
        try:
            readings = {}
            
            # Parsing selon la documentation Alicat
            # Byte 0-1: Gas number (UINT)
            readings['gas_number'] = struct.unpack('<H', data[0:2])[0]
            
            # Byte 2-3: Alarm outputs state (UINT)
            readings['alarm_state'] = struct.unpack('<H', data[2:4])[0]
            
            # Byte 4-7: Device status (UDINT)
            readings['device_status'] = struct.unpack('<I', data[4:8])[0]
            
            # Byte 8-11: Current setpoint (REAL)
            readings['setpoint'] = struct.unpack('<f', data[8:12])[0]
            
            # Byte 12-15: Current valve drive (REAL)
            readings['valve_drive'] = struct.unpack('<f', data[12:16])[0]
            
            # Byte 16-19: Primary pressure (REAL)
            readings['pressure_primary'] = struct.unpack('<f', data[16:20])[0]
            
            # Byte 20-23: Secondary pressure (REAL)
            readings['pressure_secondary'] = struct.unpack('<f', data[20:24])[0]
            
            # Byte 24-27: Barometric pressure (REAL)
            readings['pressure_barometric'] = struct.unpack('<f', data[24:28])[0]
            
            # Byte 28-31: Temperature (REAL)
            readings['temperature'] = struct.unpack('<f', data[28:32])[0]
            
            # Byte 32-35: Volumetric flow (REAL)
            readings['flow_volumetric'] = struct.unpack('<f', data[32:36])[0]
            
            # Byte 36-39: Mass flow (REAL)
            readings['flow_mass'] = struct.unpack('<f', data[36:40])[0]
            
            # Byte 40-43: Totalizer 1 (REAL)
            readings['totalizer_1'] = struct.unpack('<f', data[40:44])[0]
            
            # Byte 44-47: Totalizer 2 (REAL)
            readings['totalizer_2'] = struct.unpack('<f', data[44:48])[0]
            
            # Byte 48-51: Humidity (REAL)
            readings['humidity'] = struct.unpack('<f', data[48:52])[0]
            
            return readings
            
        except Exception as e:
            print(f"Erreur parsing lectures: {e}")
            print(f"Données reçues ({len(data)} bytes): {data.hex()}")
            return None
    
    def get_pressure(self):
        try :
            readings = self.get_readings()
            pres = readings['pressure_primary']
            return pres
        except :
            return 0

    # ── PID ──────────────────────────────────────────────────────────────────
    # Assembly 109 : Command Request  (UDINT cmd_id + DINT argument)
    # Assembly 110 : Command Result   (UDINT id, DINT arg, UDINT status, DINT retval)
    # IDs : 0=NOP, 8=set P, 9=set D, 10=set I, 13=set algo, 14=read gain
    # Algo : 1=PDF, 2=PD2I  |  Gain idx : 0=P, 1=D, 2=I

    _CMD_ASSEMBLY    = 109
    _RESULT_ASSEMBLY = 110
    _STATUS_SUCCESS  = 0
    _STATUS_PROGRESS = 1

    def _send_command(self, cmd_id: int, argument: int = 0,
                      timeout: float = 1.0) -> 'dict | None':
        """Envoie une commande CIP et lit le résultat dans Assembly 110."""
        data = struct.pack('<Ii', cmd_id, argument)
        if not self.write_assembly(self._CMD_ASSEMBLY, data):
            return None
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self.read_assembly(self._RESULT_ASSEMBLY)
            if raw and len(raw) >= 16:
                result_id = struct.unpack('<I', raw[0:4])[0]
                status    = struct.unpack('<I', raw[8:12])[0]
                ret_val   = struct.unpack('<i', raw[12:16])[0]
                if result_id == cmd_id and status != self._STATUS_PROGRESS:
                    return {'status': status, 'return_value': ret_val}
            time.sleep(0.05)
        return None

    def _nop(self):
        """NOP — sépare des commandes identiques consécutives."""
        self.write_assembly(self._CMD_ASSEMBLY, struct.pack('<Ii', 0, 0))
        time.sleep(0.05)

    def _read_gain(self, gain_idx: int) -> 'int | None':
        """Lit un gain PID. gain_idx: 0=P, 1=D, 2=I."""
        result = self._send_command(14, gain_idx)
        if result and result['status'] == self._STATUS_SUCCESS:
            return result['return_value']
        return None

    def get_pid(self) -> 'dict | None':
        """
        Lit les gains PID courants.
        Retourne {'loop_type': 'PD/PDF'|'PD2I', 'P': int, 'D': int, 'I': int}
        """
        try:
            p = self._read_gain(0); self._nop()
            d = self._read_gain(1); self._nop()
            i = self._read_gain(2)
            if p is None or d is None or i is None:
                return None
            loop_type = getattr(self, '_last_algo', 'PD/PDF')
            return {'loop_type': loop_type, 'P': p, 'D': d, 'I': i}
        except Exception as e:
            print(f"[get_pid] {e}")
            return None

    def set_pid(self, P: int = None, D: int = None, I: int = None,
                loop_type: str = 'PD/PDF') -> bool:
        """
        Écrit les gains PID et sélectionne l'algorithme.
        P, D, I : entiers 0–65535 (None = ne pas modifier)
        loop_type : 'PD/PDF' ou 'PD2I'
        """
        try:
            algo = 2 if loop_type == 'PD2I' else 1
            r = self._send_command(13, algo)
            if not r or r['status'] != self._STATUS_SUCCESS:
                return False
            self._last_algo = loop_type
            self._nop()

            for cmd_id, val in [(8, P), (9, D), (10, I)]:
                if val is None:
                    continue
                if cmd_id == 10 and loop_type != 'PD2I':
                    continue   # I ignoré en mode PDF
                r = self._send_command(cmd_id, int(val))
                if not r or r['status'] != self._STATUS_SUCCESS:
                    return False
                self._nop()
            return True
        except Exception as e:
            print(f"[set_pid] {e}")
            return False


    def display_readings(self, readings: Dict[str, Any]):
        """Affiche les lectures de manière formatée"""
        if not readings:
            print("Aucune lecture disponible")
            return
        
        print("\n=== LECTURES ALICAT ===")
        print(f"Gas Number: {readings['gas_number']}")
        print(f"Alarm State: 0x{readings['alarm_state']:04X}")
        print(f"Device Status: 0x{readings['device_status']:08X}")
        print(f"Setpoint: {readings['setpoint']:.3f}")
        print(f"Valve Drive: {readings['valve_drive']:.1f}%")
        print(f"Pressure Primary: {readings['pressure_primary']:.3f}")
        print(f"Pressure Secondary: {readings['pressure_secondary']:.3f}")
        print(f"Pressure Barometric: {readings['pressure_barometric']:.3f}")
        print(f"Temperature: {readings['temperature']:.2f}°C")
        print(f"Flow Volumetric: {readings['flow_volumetric']:.3f}")
        print(f"Flow Mass: {readings['flow_mass']:.3f}")
        print(f"Totalizer 1: {readings['totalizer_1']:.3f}")
        print(f"Totalizer 2: {readings['totalizer_2']:.3f}")
        print(f"Humidity: {readings['humidity']:.1f}%")
    
    
    
    def test_setpoint_control(self, setpoints: list, hold_time: float = 5.0):
        """
        Test de contrôle du setpoint avec différentes valeurs
        
        Args:
            setpoints: Liste des setpoints à tester
            hold_time: Temps d'attente à chaque setpoint
        """
        print(f"\n=== TEST CONTRÔLE SETPOINT ===")
        
        for i, setpoint in enumerate(setpoints):
            print(f"\nÉtape {i+1}/{len(setpoints)}: Setpoint = {setpoint}")
            
            # Définition du setpoint
            if self.set_setpoint(setpoint):
                # Attente et monitoring
                start_time = time.time()
                while time.time() - start_time < hold_time:
                    readings = self.get_readings()
                    if readings:
                        print(f"  Flow: {readings['flow_mass']:.3f}, "
                              f"Setpoint actuel: {readings['setpoint']:.3f}, "
                              f"Valve: {readings['valve_drive']:.1f}%")
                    time.sleep(1.0)
            else:
                print(f"  ✗ Échec définition setpoint {setpoint}")
    
    def get_device_info(self) -> Optional[Dict[str, Any]]:
        """Récupère les informations d'identité du dispositif"""
        try:
            # Identity Object - Get All Attributes
            result = self.driver.generic_message(
                service=0x01,  # Get_Attributes_All
                class_code=1,  # Identity Object
                instance=1
            )
            
            if result and hasattr(result, 'value'):
                data = result.value
                if len(data) >= 14:
                    info = {}
                    info['vendor_id'] = struct.unpack('<H', data[0:2])[0]
                    info['device_type'] = struct.unpack('<H', data[2:4])[0]
                    info['product_code'] = struct.unpack('<H', data[4:6])[0]
                    info['status'] = struct.unpack('<H', data[8:10])[0]
                    info['serial_number'] = struct.unpack('<I', data[10:14])[0]
                    
                    # Product name
                    if len(data) > 14:
                        str_len = data[14]
                        if len(data) > 14 + str_len:
                            info['product_name'] = data[15:15+str_len].decode('utf-8', errors='ignore')
                    
                    return info
            return None
            
        except Exception as e:
            print(f"Erreur lecture info dispositif: {e}")
            return None

def main():
    """Exemple d'utilisation complète"""
    
    alicat_ip = "10.0.1.101"
    
    alicat = AlicatController(alicat_ip)
    
    try:
        # Connexion
        if not alicat.connect():
            return
        
        # Informations dispositif
        print("=== INFORMATIONS DISPOSITIF ===")
        device_info = alicat.get_device_info()
        if device_info:
            print(f"Vendor ID: {device_info['vendor_id']} ({'Alicat' if device_info['vendor_id'] == 1174 else 'Autre'})")
            print(f"Product: {device_info.get('product_name', 'N/A')}")
            print(f"Serial: {device_info['serial_number']}")
        print('Presure',alicat.get_pressure())
        # Lecture des données actuelles
        print("\n=== LECTURES ACTUELLES ===")
        readings = alicat.get_readings()
        if readings:
            alicat.display_readings(readings)
        else:
            print("Impossible de lire les données")
            return
        
        # Test de modification du setpoint
        print("\n=== TEST SETPOINT ===")
        current_setpoint = readings['setpoint']
        print(f"Setpoint actuel: {current_setpoint}")
        
        # Test avec un nouveau setpoint
        new_setpoint = 20.0  # Ajustez selon vos besoins
        if alicat.set_setpoint(new_setpoint):
            time.sleep(1)  # Attendre la mise à jour
            
            # Vérification
            readings = alicat.get_readings()
            if readings:
                print(f"Nouveau setpoint: {readings['setpoint']}")
        
    except Exception as e:
        print(f"Erreur: {e}")
    finally:
        alicat.disconnect()

if __name__ == "__main__":
    main()