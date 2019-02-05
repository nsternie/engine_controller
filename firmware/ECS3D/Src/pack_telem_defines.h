/// pack_telem_defines.h
/// Last autogenerated: Sun Feb  3 16:57:27 2019

#include "globals.h"
#include "calibrations.h"
#include "config.h"

#define	TELEM_ITEM_0	((uint32_t) (valve_states*1)) >> 0 
#define	TELEM_ITEM_1	((uint32_t) (valve_states*1)) >> 8 
#define	TELEM_ITEM_2	((uint32_t) (valve_states*1)) >> 16 
#define	TELEM_ITEM_3	((uint32_t) (valve_states*1)) >> 24 
#define	TELEM_ITEM_4	((int16_t) (pressure[0]*1)) >> 0 
#define	TELEM_ITEM_5	((int16_t) (pressure[0]*1)) >> 8 
#define	TELEM_ITEM_6	((int16_t) (pressure[1]*1)) >> 0 
#define	TELEM_ITEM_7	((int16_t) (pressure[1]*1)) >> 8 
#define	TELEM_ITEM_8	((int16_t) (pressure[2]*1)) >> 0 
#define	TELEM_ITEM_9	((int16_t) (pressure[2]*1)) >> 8 
#define	TELEM_ITEM_10	((int16_t) (pressure[3]*1)) >> 0 
#define	TELEM_ITEM_11	((int16_t) (pressure[3]*1)) >> 8 
#define	TELEM_ITEM_12	((int16_t) (pressure[4]*1)) >> 0 
#define	TELEM_ITEM_13	((int16_t) (pressure[4]*1)) >> 8 
#define	TELEM_ITEM_14	((int16_t) (pressure[5]*1)) >> 0 
#define	TELEM_ITEM_15	((int16_t) (pressure[5]*1)) >> 8 
#define	TELEM_ITEM_16	((int16_t) (pressure[6]*1)) >> 0 
#define	TELEM_ITEM_17	((int16_t) (pressure[6]*1)) >> 8 
#define	TELEM_ITEM_18	((int16_t) (pressure[7]*1)) >> 0 
#define	TELEM_ITEM_19	((int16_t) (pressure[7]*1)) >> 8 
#define	TELEM_ITEM_20	((int16_t) (pressure[8]*2)) >> 0 
#define	TELEM_ITEM_21	((int16_t) (pressure[8]*2)) >> 8 
#define	TELEM_ITEM_22	((int16_t) (pressure[9]*3)) >> 0 
#define	TELEM_ITEM_23	((int16_t) (pressure[9]*3)) >> 8 
#define	TELEM_ITEM_24	((int16_t) (pressure[10]*4)) >> 0 
#define	TELEM_ITEM_25	((int16_t) (pressure[10]*4)) >> 8 
#define	TELEM_ITEM_26	((int16_t) (pressure[11]*5)) >> 0 
#define	TELEM_ITEM_27	((int16_t) (pressure[11]*5)) >> 8 
#define	TELEM_ITEM_28	((int16_t) (pressure[12]*6)) >> 0 
#define	TELEM_ITEM_29	((int16_t) (pressure[12]*6)) >> 8 
#define	TELEM_ITEM_30	((int16_t) (pressure[13]*7)) >> 0 
#define	TELEM_ITEM_31	((int16_t) (pressure[13]*7)) >> 8 
#define	TELEM_ITEM_32	((int16_t) (pressure[14]*8)) >> 0 
#define	TELEM_ITEM_33	((int16_t) (pressure[14]*8)) >> 8 
#define	TELEM_ITEM_34	((int16_t) (pressure[15]*9)) >> 0 
#define	TELEM_ITEM_35	((int16_t) (pressure[15]*9)) >> 8 
#define	TELEM_ITEM_36	((int16_t) (pressure[16]*10)) >> 0 
#define	TELEM_ITEM_37	((int16_t) (pressure[16]*10)) >> 8 
#define	TELEM_ITEM_38	((int16_t) (pressure[17]*11)) >> 0 
#define	TELEM_ITEM_39	((int16_t) (pressure[17]*11)) >> 8 
#define	TELEM_ITEM_40	((int16_t) (pressure[18]*12)) >> 0 
#define	TELEM_ITEM_41	((int16_t) (pressure[18]*12)) >> 8 
#define	TELEM_ITEM_42	((int16_t) (pressure[19]*13)) >> 0 
#define	TELEM_ITEM_43	((int16_t) (pressure[19]*13)) >> 8 
#define	TELEM_ITEM_44	((int16_t) (pressure[20]*14)) >> 0 
#define	TELEM_ITEM_45	((int16_t) (pressure[20]*14)) >> 8 
#define	TELEM_ITEM_46	((int16_t) (pressure[21]*15)) >> 0 
#define	TELEM_ITEM_47	((int16_t) (pressure[21]*15)) >> 8 
#define	TELEM_ITEM_48	((uint32_t) (samplerate*1)) >> 0 
#define	TELEM_ITEM_49	((uint32_t) (samplerate*1)) >> 8 
#define	TELEM_ITEM_50	((uint32_t) (samplerate*1)) >> 16 
#define	TELEM_ITEM_51	((uint32_t) (samplerate*1)) >> 24 
#define	TELEM_ITEM_52	((uint16_t) (main_cycle_time[0]*1)) >> 0 
#define	TELEM_ITEM_53	((uint16_t) (main_cycle_time[0]*1)) >> 8 
#define	TELEM_ITEM_54	((uint16_t) (motor_cycle_time[0]*1)) >> 0 
#define	TELEM_ITEM_55	((uint16_t) (motor_cycle_time[0]*1)) >> 8 
#define	TELEM_ITEM_56	((uint16_t) (adc_cycle_time[0]*1)) >> 0 
#define	TELEM_ITEM_57	((uint16_t) (adc_cycle_time[0]*1)) >> 8 
#define	TELEM_ITEM_58	((uint32_t) (telemetry_cycle_time[0]*1)) >> 0 
#define	TELEM_ITEM_59	((uint32_t) (telemetry_cycle_time[0]*1)) >> 8 
#define	TELEM_ITEM_60	((uint32_t) (telemetry_cycle_time[0]*1)) >> 16 
#define	TELEM_ITEM_61	((uint32_t) (telemetry_cycle_time[0]*1)) >> 24 
#define	TELEM_ITEM_62	((int16_t) (ebatt*1000)) >> 0 
#define	TELEM_ITEM_63	((int16_t) (ebatt*1000)) >> 8 
#define	TELEM_ITEM_64	((int16_t) (ibus*100)) >> 0 
#define	TELEM_ITEM_65	((int16_t) (ibus*100)) >> 8 
#define	TELEM_ITEM_66	((uint16_t) (telemetry_rate[0]*1)) >> 0 
#define	TELEM_ITEM_67	((uint16_t) (telemetry_rate[0]*1)) >> 8 
#define	TELEM_ITEM_68	((uint8_t) (STATE*1)) >> 0 
#define	TELEM_ITEM_69	((int16_t) (ivlv[0]*100)) >> 0 
#define	TELEM_ITEM_70	((int16_t) (ivlv[0]*100)) >> 8 
#define	TELEM_ITEM_71	((int16_t) (ivlv[1]*100)) >> 0 
#define	TELEM_ITEM_72	((int16_t) (ivlv[1]*100)) >> 8 
#define	TELEM_ITEM_73	((int16_t) (ivlv[2]*100)) >> 0 
#define	TELEM_ITEM_74	((int16_t) (ivlv[2]*100)) >> 8 
#define	TELEM_ITEM_75	((int16_t) (ivlv[3]*100)) >> 0 
#define	TELEM_ITEM_76	((int16_t) (ivlv[3]*100)) >> 8 
#define	TELEM_ITEM_77	((int16_t) (ivlv[4]*100)) >> 0 
#define	TELEM_ITEM_78	((int16_t) (ivlv[4]*100)) >> 8 
#define	TELEM_ITEM_79	((int16_t) (ivlv[5]*100)) >> 0 
#define	TELEM_ITEM_80	((int16_t) (ivlv[5]*100)) >> 8 
#define	TELEM_ITEM_81	((int16_t) (ivlv[6]*100)) >> 0 
#define	TELEM_ITEM_82	((int16_t) (ivlv[6]*100)) >> 8 
#define	TELEM_ITEM_83	((int16_t) (ivlv[7]*100)) >> 0 
#define	TELEM_ITEM_84	((int16_t) (ivlv[7]*100)) >> 8 
#define	TELEM_ITEM_85	((int16_t) (ivlv[8]*100)) >> 0 
#define	TELEM_ITEM_86	((int16_t) (ivlv[8]*100)) >> 8 
#define	TELEM_ITEM_87	((int16_t) (ivlv[9]*100)) >> 0 
#define	TELEM_ITEM_88	((int16_t) (ivlv[9]*100)) >> 8 
#define	TELEM_ITEM_89	((int16_t) (ivlv[10]*100)) >> 0 
#define	TELEM_ITEM_90	((int16_t) (ivlv[10]*100)) >> 8 
#define	TELEM_ITEM_91	((int16_t) (ivlv[11]*100)) >> 0 
#define	TELEM_ITEM_92	((int16_t) (ivlv[11]*100)) >> 8 
#define	TELEM_ITEM_93	((int16_t) (ivlv[12]*100)) >> 0 
#define	TELEM_ITEM_94	((int16_t) (ivlv[12]*100)) >> 8 
#define	TELEM_ITEM_95	((int16_t) (ivlv[13]*100)) >> 0 
#define	TELEM_ITEM_96	((int16_t) (ivlv[13]*100)) >> 8 
#define	TELEM_ITEM_97	((int16_t) (ivlv[14]*100)) >> 0 
#define	TELEM_ITEM_98	((int16_t) (ivlv[14]*100)) >> 8 
#define	TELEM_ITEM_99	((int16_t) (ivlv[15]*100)) >> 0 
#define	TELEM_ITEM_100	((int16_t) (ivlv[15]*100)) >> 8 
#define	TELEM_ITEM_101	((int16_t) (ivlv[16]*100)) >> 0 
#define	TELEM_ITEM_102	((int16_t) (ivlv[16]*100)) >> 8 
#define	TELEM_ITEM_103	((int16_t) (ivlv[17]*100)) >> 0 
#define	TELEM_ITEM_104	((int16_t) (ivlv[17]*100)) >> 8 
#define	TELEM_ITEM_105	((int16_t) (ivlv[18]*100)) >> 0 
#define	TELEM_ITEM_106	((int16_t) (ivlv[18]*100)) >> 8 
#define	TELEM_ITEM_107	((int16_t) (ivlv[19]*100)) >> 0 
#define	TELEM_ITEM_108	((int16_t) (ivlv[19]*100)) >> 8 
#define	TELEM_ITEM_109	((int16_t) (ivlv[20]*100)) >> 0 
#define	TELEM_ITEM_110	((int16_t) (ivlv[20]*100)) >> 8 
#define	TELEM_ITEM_111	((int16_t) (ivlv[21]*100)) >> 0 
#define	TELEM_ITEM_112	((int16_t) (ivlv[21]*100)) >> 8 
#define	TELEM_ITEM_113	((int16_t) (ivlv[22]*100)) >> 0 
#define	TELEM_ITEM_114	((int16_t) (ivlv[22]*100)) >> 8 
#define	TELEM_ITEM_115	((int16_t) (ivlv[23]*100)) >> 0 
#define	TELEM_ITEM_116	((int16_t) (ivlv[23]*100)) >> 8 
#define	TELEM_ITEM_117	((int16_t) (ivlv[24]*100)) >> 0 
#define	TELEM_ITEM_118	((int16_t) (ivlv[24]*100)) >> 8 
#define	TELEM_ITEM_119	((int16_t) (ivlv[25]*100)) >> 0 
#define	TELEM_ITEM_120	((int16_t) (ivlv[25]*100)) >> 8 
#define	TELEM_ITEM_121	((int16_t) (ivlv[26]*100)) >> 0 
#define	TELEM_ITEM_122	((int16_t) (ivlv[26]*100)) >> 8 
#define	TELEM_ITEM_123	((int16_t) (ivlv[27]*100)) >> 0 
#define	TELEM_ITEM_124	((int16_t) (ivlv[27]*100)) >> 8 
#define	TELEM_ITEM_125	((int16_t) (ivlv[28]*100)) >> 0 
#define	TELEM_ITEM_126	((int16_t) (ivlv[28]*100)) >> 8 
#define	TELEM_ITEM_127	((int16_t) (ivlv[29]*100)) >> 0 
#define	TELEM_ITEM_128	((int16_t) (ivlv[29]*100)) >> 8 
#define	TELEM_ITEM_129	((int16_t) (ivlv[30]*100)) >> 0 
#define	TELEM_ITEM_130	((int16_t) (ivlv[30]*100)) >> 8 
#define	TELEM_ITEM_131	((int16_t) (ivlv[31]*100)) >> 0 
#define	TELEM_ITEM_132	((int16_t) (ivlv[31]*100)) >> 8 
#define	TELEM_ITEM_133	((int16_t) (evlv[0]*100)) >> 0 
#define	TELEM_ITEM_134	((int16_t) (evlv[0]*100)) >> 8 
#define	TELEM_ITEM_135	((int16_t) (evlv[1]*100)) >> 0 
#define	TELEM_ITEM_136	((int16_t) (evlv[1]*100)) >> 8 
#define	TELEM_ITEM_137	((int16_t) (evlv[2]*100)) >> 0 
#define	TELEM_ITEM_138	((int16_t) (evlv[2]*100)) >> 8 
#define	TELEM_ITEM_139	((int16_t) (evlv[3]*100)) >> 0 
#define	TELEM_ITEM_140	((int16_t) (evlv[3]*100)) >> 8 
#define	TELEM_ITEM_141	((int16_t) (evlv[4]*100)) >> 0 
#define	TELEM_ITEM_142	((int16_t) (evlv[4]*100)) >> 8 
#define	TELEM_ITEM_143	((int16_t) (evlv[5]*100)) >> 0 
#define	TELEM_ITEM_144	((int16_t) (evlv[5]*100)) >> 8 
#define	TELEM_ITEM_145	((int16_t) (evlv[6]*100)) >> 0 
#define	TELEM_ITEM_146	((int16_t) (evlv[6]*100)) >> 8 
#define	TELEM_ITEM_147	((int16_t) (evlv[7]*100)) >> 0 
#define	TELEM_ITEM_148	((int16_t) (evlv[7]*100)) >> 8 
#define	TELEM_ITEM_149	((int16_t) (evlv[8]*100)) >> 0 
#define	TELEM_ITEM_150	((int16_t) (evlv[8]*100)) >> 8 
#define	TELEM_ITEM_151	((int16_t) (evlv[9]*100)) >> 0 
#define	TELEM_ITEM_152	((int16_t) (evlv[9]*100)) >> 8 
#define	TELEM_ITEM_153	((int16_t) (evlv[10]*100)) >> 0 
#define	TELEM_ITEM_154	((int16_t) (evlv[10]*100)) >> 8 
#define	TELEM_ITEM_155	((int16_t) (evlv[11]*100)) >> 0 
#define	TELEM_ITEM_156	((int16_t) (evlv[11]*100)) >> 8 
#define	TELEM_ITEM_157	((int16_t) (evlv[12]*100)) >> 0 
#define	TELEM_ITEM_158	((int16_t) (evlv[12]*100)) >> 8 
#define	TELEM_ITEM_159	((int16_t) (evlv[13]*100)) >> 0 
#define	TELEM_ITEM_160	((int16_t) (evlv[13]*100)) >> 8 
#define	TELEM_ITEM_161	((int16_t) (evlv[14]*100)) >> 0 
#define	TELEM_ITEM_162	((int16_t) (evlv[14]*100)) >> 8 
#define	TELEM_ITEM_163	((int16_t) (evlv[15]*100)) >> 0 
#define	TELEM_ITEM_164	((int16_t) (evlv[15]*100)) >> 8 
#define	TELEM_ITEM_165	((int16_t) (evlv[16]*100)) >> 0 
#define	TELEM_ITEM_166	((int16_t) (evlv[16]*100)) >> 8 
#define	TELEM_ITEM_167	((int16_t) (evlv[17]*100)) >> 0 
#define	TELEM_ITEM_168	((int16_t) (evlv[17]*100)) >> 8 
#define	TELEM_ITEM_169	((int16_t) (evlv[18]*100)) >> 0 
#define	TELEM_ITEM_170	((int16_t) (evlv[18]*100)) >> 8 
#define	TELEM_ITEM_171	((int16_t) (evlv[19]*100)) >> 0 
#define	TELEM_ITEM_172	((int16_t) (evlv[19]*100)) >> 8 
#define	TELEM_ITEM_173	((int16_t) (evlv[20]*100)) >> 0 
#define	TELEM_ITEM_174	((int16_t) (evlv[20]*100)) >> 8 
#define	TELEM_ITEM_175	((int16_t) (evlv[21]*100)) >> 0 
#define	TELEM_ITEM_176	((int16_t) (evlv[21]*100)) >> 8 
#define	TELEM_ITEM_177	((int16_t) (evlv[22]*100)) >> 0 
#define	TELEM_ITEM_178	((int16_t) (evlv[22]*100)) >> 8 
#define	TELEM_ITEM_179	((int16_t) (evlv[23]*100)) >> 0 
#define	TELEM_ITEM_180	((int16_t) (evlv[23]*100)) >> 8 
#define	TELEM_ITEM_181	((int16_t) (evlv[24]*100)) >> 0 
#define	TELEM_ITEM_182	((int16_t) (evlv[24]*100)) >> 8 
#define	TELEM_ITEM_183	((int16_t) (evlv[25]*100)) >> 0 
#define	TELEM_ITEM_184	((int16_t) (evlv[25]*100)) >> 8 
#define	TELEM_ITEM_185	((int16_t) (evlv[26]*100)) >> 0 
#define	TELEM_ITEM_186	((int16_t) (evlv[26]*100)) >> 8 
#define	TELEM_ITEM_187	((int16_t) (evlv[27]*100)) >> 0 
#define	TELEM_ITEM_188	((int16_t) (evlv[27]*100)) >> 8 
#define	TELEM_ITEM_189	((int16_t) (evlv[28]*100)) >> 0 
#define	TELEM_ITEM_190	((int16_t) (evlv[28]*100)) >> 8 
#define	TELEM_ITEM_191	((int16_t) (evlv[29]*100)) >> 0 
#define	TELEM_ITEM_192	((int16_t) (evlv[29]*100)) >> 8 
#define	TELEM_ITEM_193	((int16_t) (evlv[30]*100)) >> 0 
#define	TELEM_ITEM_194	((int16_t) (evlv[30]*100)) >> 8 
#define	TELEM_ITEM_195	((int16_t) (evlv[31]*100)) >> 0 
#define	TELEM_ITEM_196	((int16_t) (evlv[31]*100)) >> 8 
#define	TELEM_ITEM_197	((int32_t) (e3v*100)) >> 0 
#define	TELEM_ITEM_198	((int32_t) (e3v*100)) >> 8 
#define	TELEM_ITEM_199	((int32_t) (e3v*100)) >> 16 
#define	TELEM_ITEM_200	((int32_t) (e3v*100)) >> 24 
#define	TELEM_ITEM_201	((int16_t) (e28v*100)) >> 0 
#define	TELEM_ITEM_202	((int16_t) (e28v*100)) >> 8 
#define	TELEM_ITEM_203	((int32_t) (e5v*100)) >> 0 
#define	TELEM_ITEM_204	((int32_t) (e5v*100)) >> 8 
#define	TELEM_ITEM_205	((int32_t) (e5v*100)) >> 16 
#define	TELEM_ITEM_206	((int32_t) (e5v*100)) >> 24 
#define	TELEM_ITEM_207	((uint16_t) (BOARD_ID*1)) >> 0 
#define	TELEM_ITEM_208	((uint16_t) (BOARD_ID*1)) >> 8 
#define	TELEM_ITEM_209	((uint16_t) (last_packet_number*1)) >> 0 
#define	TELEM_ITEM_210	((uint16_t) (last_packet_number*1)) >> 8 
#define	TELEM_ITEM_211	((uint16_t) (last_command_id*1)) >> 0 
#define	TELEM_ITEM_212	((uint16_t) (last_command_id*1)) >> 8 
#define	TELEM_ITEM_213	((uint16_t) (tbrd*10)) >> 0 
#define	TELEM_ITEM_214	((uint16_t) (tbrd*10)) >> 8 
#define	TELEM_ITEM_215	((uint16_t) (tvlv*10)) >> 0 
#define	TELEM_ITEM_216	((uint16_t) (tvlv*10)) >> 8 
#define	TELEM_ITEM_217	((uint16_t) (tmtr*10)) >> 0 
#define	TELEM_ITEM_218	((uint16_t) (tmtr*10)) >> 8 
#define	TELEM_ITEM_219	((uint8_t) (error_code*1)) >> 0 
#define	TELEM_ITEM_220	((uint8_t) (LOGGING_ACTIVE*1)) >> 0 
#define	TELEM_ITEM_221	((uint16_t) (logfile->current_page*1)) >> 0 
#define	TELEM_ITEM_222	((uint16_t) (logfile->current_page*1)) >> 8 
#define	TELEM_ITEM_223	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_224	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_225	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_226	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_227	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_228	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_229	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_230	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_231	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_232	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_233	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_234	((uint8_t) (0*1)) >> 0 
#define	TELEM_ITEM_235	0
#define	TELEM_ITEM_236	0
#define	TELEM_ITEM_237	0
#define	TELEM_ITEM_238	0
#define	TELEM_ITEM_239	0
#define	TELEM_ITEM_240	0
#define	TELEM_ITEM_241	0
#define	TELEM_ITEM_242	0
#define	TELEM_ITEM_243	0
#define	TELEM_ITEM_244	0
#define	TELEM_ITEM_245	0
#define	TELEM_ITEM_246	0
#define	TELEM_ITEM_247	0
#define	TELEM_ITEM_248	0
#define	TELEM_ITEM_249	0
#define	TELEM_ITEM_250	0
#define	TELEM_ITEM_251	0
#define	TELEM_ITEM_252	0
#define	TELEM_ITEM_253	0
#define	PACKET_SIZE	235
