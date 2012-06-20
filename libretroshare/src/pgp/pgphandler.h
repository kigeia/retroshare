#pragma once

// This class implements an abstract pgp handler to be used in RetroShare.
//
#include <stdint.h>
#include <string>
#include <list>
#include <map>
#include <set>
#include <util/rsthreads.h>
#include <util/rsid.h>

extern "C" {
#include <openpgpsdk/types.h>
#include <openpgpsdk/keyring.h>
#include <openpgpsdk/keyring_local.h>
}

static const int KEY_ID_SIZE          =  8 ;
static const int KEY_FINGERPRINT_SIZE = 20 ;

typedef std::string (*PassphraseCallback)(void *data, const char *uid_hint, const char *passphrase_info, int prev_was_bad) ;

typedef t_RsGenericIdType<KEY_ID_SIZE> 			 PGPIdType;
typedef t_RsGenericIdType<KEY_FINGERPRINT_SIZE>  PGPFingerprintType ;

class PGPCertificateInfo
{
	public:
		PGPCertificateInfo() {}

		std::string _name;
		std::string _email;
		std::string _comment;

		std::set<std::string> signers;

		uint32_t _trustLvl;
		uint32_t _validLvl;
		uint32_t _flags ;

		PGPFingerprintType _fpr;           /* fingerprint */
		PGPIdType          _key_id ;

		uint32_t _key_index ;			// index to array of keys in the public keyring 

		static const uint32_t PGP_CERTIFICATE_FLAG_ACCEPT_CONNEXION      = 0x0001 ;
		static const uint32_t PGP_CERTIFICATE_FLAG_HAS_OWN_SIGNATURE     = 0x0002 ;
		static const uint32_t PGP_CERTIFICATE_FLAG_HAS_SIGNED_ME         = 0x0004 ;
		static const uint32_t PGP_CERTIFICATE_FLAG_UNSUPPORTED_ALGORITHM = 0x0008 ;	// set when the key is not RSA, so that RS avoids to use it.
};

class PGPHandler
{
	public:
		PGPHandler(	const std::string& path_to_public_keyring, 
						const std::string& path_to_secret_keyring, 
						const std::string& path_to_trust_database, 
						const std::string& pgp_lock_file) ;

		virtual ~PGPHandler() ;

		/**
		 * @param ids list of gpg certificate ids (note, not the actual certificates)
		 */
		bool getGPGFilteredList(std::list<PGPIdType>& list,bool (*filter)(const PGPCertificateInfo&) = NULL) const ;

		bool availableGPGCertificatesWithPrivateKeys(std::list<PGPIdType>& ids);
		bool GeneratePGPCertificate(const std::string& name, const std::string& email, const std::string& passwd, PGPIdType& pgpId, std::string& errString) ;

		bool LoadCertificateFromString(const std::string& pem, PGPIdType& gpg_id, std::string& error_string);
		std::string SaveCertificateToString(const PGPIdType& id,bool include_signatures) ;

		bool TrustCertificate(const PGPIdType& id, int trustlvl);

		bool SignDataBin(const PGPIdType& id,const void *data, const uint32_t len, unsigned char *sign, unsigned int *signlen) ;
		bool VerifySignBin(const void *data, uint32_t data_len, unsigned char *sign, unsigned int sign_len, const PGPFingerprintType& withfingerprint) ;

		bool encryptTextToFile(const PGPIdType& key_id,const std::string& text,const std::string& outfile) ;
		bool decryptTextFromFile(const PGPIdType& key_id,std::string& text,const std::string& inputfile) ;

		bool getKeyFingerprint(const PGPIdType& id,PGPFingerprintType& fp) const ;
		void setAcceptConnexion(const PGPIdType&,bool) ;

		bool isKeySupported(const PGPIdType& id) const ;

		void privateTrustCertificate(const PGPIdType& id,int valid_level) ;	

		// Write keyring
		bool publicKeyringChanged() const { return _pubring_changed ; }
		bool secretKeyringChanged() const { return _secring_changed ; }

		bool writeSecretKeyring() ;
		bool writePublicKeyring() ;

		const PGPCertificateInfo *getCertificateInfo(const PGPIdType& id) const ;

		bool isGPGId(const std::string &id);
		bool isGPGSigned(const std::string &id);
		bool isGPGAccepted(const std::string &id);

		static void setPassphraseCallback(PassphraseCallback cb) ;
		static PassphraseCallback passphraseCallback() { return _passphrase_callback ; }

		// Debug stuff.
		virtual bool printKeys() const ;

	private:
		void initCertificateInfo(PGPCertificateInfo& cert,const ops_keydata_t *keydata,uint32_t i) ;
		void validateAndUpdateSignatures(PGPCertificateInfo& cert,const ops_keydata_t *keydata) ;

		const ops_keydata_t *getPublicKey(const PGPIdType&) const ;
		const ops_keydata_t *getSecretKey(const PGPIdType&) const ;

		void locked_readPrivateTrustDatabase() ;
		void locked_writePrivateTrustDatabase() ;

		// Members.
		//
		RsMutex pgphandlerMtx ;

		ops_keyring_t *_pubring ;
		ops_keyring_t *_secring ;

		std::map<std::string,PGPCertificateInfo> _public_keyring_map ;	// used for fast access to keys. Gives the index in the keyring.
		std::map<std::string,PGPCertificateInfo> _secret_keyring_map ;

		const std::string _pubring_path ;
		const std::string _secring_path ;
		const std::string _trustdb_path ;
		const std::string _pgp_lock_filename ;

		bool _pubring_changed ;
		bool _secring_changed ;
		bool _trustdb_changed ;

		// Helper functions.
		//
		static std::string makeRadixEncodedPGPKey(const ops_keydata_t *key) ;
		static ops_keyring_t *allocateOPSKeyring() ;
		static void addNewKeyToOPSKeyring(ops_keyring_t*, const ops_keydata_t&) ;
		static PassphraseCallback _passphrase_callback ;
		static bool mergeKeySignatures(ops_keydata_t *dst,const ops_keydata_t *src) ;	// returns true if signature lists are different
};

