import numpy as np
import cv2
from sklearn import svm
from collections import Counter
from sklearn.externals import joblib
from sklearn import svm
from sklearn import preprocessing
from sklearn.ensemble import ExtraTreesClassifier


class Identifier:
	def __init__(self,grabable = set([]),clf = None):
		self.grabable = grabable #TODO if we care to, not used at the mo
		self.orb = orb = cv2.ORB(nfeatures = 1000)#,nlevels = 20, scaleFactor = 1.05)
		self.items = [ "champion_copper_plus_spark_plug", "cheezit_big_original","crayola_64_ct", "dove_beauty_bar", "elmers_washable_no_run_school_glue","expo_dry_erase_board_eraser", "feline_greenies_dental_treats","first_years_take_and_toss_straw_cups", "genuine_joe_plastic_stir_sticks","highland_6539_self_stick_notes", "kong_air_dog_squeakair_tennis_ball","kong_duck_dog_toy", "kong_sitting_frog_dog_toy", "kygen_squeakin_eggs_plush_puppies","mark_twain_huckleberry_finn", "mead_index_cards","mommys_helper_outlet_plugs","munchkin_white_hot_duck_bath_toy", "one_with_nature_soap_dead_sea_mud","oreo_mega_stuf", "paper_mate_12_count_mirado_black_warrior","rollodex_mesh_collection_jumbo_pencil_cup", "safety_works_safety_glasses", "sharpie_accent_tank_style_highlighters", "stanley_66_052" ]
		if not clf:
			print "Training new classifier"
			self.clf =ExtraTreesClassifier(min_samples_split = 1,n_jobs = -1,n_estimators = 150, class_weight = 'subsample')
			X = np.ascontiguousarray(joblib.load('labels.pkl'))
			Y = np.ascontiguousarray(joblib.load('features.pkl'), dtype = np.float64)
			Y = preprocessing.scale(Y)
			self.clf.fit(Y,X)
		else:
			self.clf = clf
	def identify(self,im,possibilites):
		if im is not None:
			kpTest, desTest = self.orb.detectAndCompute(im,None)
			pred = self.clf.predict(preprocessing.scale(np.array(desTest,dtype = np.float64)))
			c = Counter(pred)
			r = [(k,c[k]) for k in sorted(set(c.keys())&possibilites, key  = lambda k: c[k],reverse = True)]
			if r:
				item = r[0][0]
				print self.items[item],
				return item
			else:
				return -1

		else:
			print "Image to recognize is None"

# Id = Identifier()
# # clf = joblib.load('./clf/clf.pkl')
# for i in ['./IMG_2215.JPG','./IMG_2209.JPG','./IMG_2218.JPG','./IMG_2219.JPG','./IMG_2220.JPG','./IMG_2212.JPG','./IMG_2202.JPG','./IMG_2203.JPG','./IMG_2204.JPG','./IMG_2205.JPG','./IMG_2207.JPG']:
# 	imgTest = cv2.imread(i,0)
# 	print i, ' : ', Id.identify(imgTest,set(range(25)))
# print '************************************************'

